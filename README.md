# evse_manager

Solar-following EV charge controller for an off-grid Victron system, running on Venus OS.

**Version 7.1.1** · needs WT32-ETH01 gateway firmware **2.0.x** (2.0.2 recommended) · EVRACING EVSE-WB rev 18 · Venus OS on a Raspberry Pi 4

The script decides every 0.25 s how much current the car may take, from the battery, the solar production and the inverter load. It charges the car on sun first and never overloads the Multi RS. Home Assistant sees it as a device called **Wallbox**.

---

## How it fits together

```
Home Assistant + Mosquitto (192.168.31.43)
    |   MQTT: you set the mode, on/off and manual current; state comes back every second
    v
Venus OS Pi (192.168.31.16)  --  /data/evse_manager/evse_manager.py
    |   reads Victron D-Bus: Multi RS 6000, SmartSolar 100/20, Fronius Primo 3.6-1
    |   reads Eastron SDM120 over a USB-RS485 stick (Modbus RTU, address 2, 9600 baud)
    |
    |   HTTP + JSON: POST /api/set once a second (command + heartbeat)
    v
WT32-ETH01 gateway (192.168.31.125, evse-gateway.local)
    |   Modbus RTU 9600 8N1, slave 1
    v
EVRACING EVSE-WB wallbox  --  Type 2 pilot signal  -->  car
```

- **Home Assistant** tells the script *what* you want (the mode).
- **The script** works out *how much* current the house can afford and sends it to the gateway every second.
- **The gateway** passes it to the wallbox, and stops charging by itself if the script goes quiet for 180 s.
- **The SDM120** measures what the car really takes. The script forwards that power and the meter's kWh counter to the gateway page and to Home Assistant.

---

## Charging modes

| Mode | Label in Home Assistant | Current | Stops below SOC (resumes at) | Battery may give the car |
|---|---|---|---|---|
| **A** | A - Automatic (solar + 8A battery) | 8–32 A | 30 % (33 %) | only what it takes to hold 8 A, plus a small drain when the battery is nearly full |
| **B** | B - Fast (25A, 5kW battery) | 8–25 A | 50 % (53 %) | up to 5 kW |
| **C** | C - Super fast (32A, 5kW battery) | 8–32 A | 50 % (53 %) | up to 5 kW |
| **D** | D - Manual current | the Manual current slider, 8–32 A | 30 % (33 %) | up to 5 kW |
| **E** | E - Charging disabled | 0 | – | – |

**Limits in every mode.** The Multi RS output stays at or below 4.7 kW (5 kW minus a 300 W reserve) and battery discharge at or below 5 kW. Protection always wins over the mode, including over mode A's 8 A floor. Above 4.7 kW only sun from the Fronius and the MPPTs can feed the car.

### Mode A in practice

| Situation | What happens | HA "Limited by" |
|---|---|---|
| Night, no sun | 8 A from the battery | mode floor |
| Weak sun | still 8 A: the sun covers what it can, the battery adds the rest | mode floor |
| Good sun | the car follows the sun, battery power settles around 0 W | solar tracking |
| Plenty of sun | capped at 32 A, the rest charges the battery | mode cap / inverter headroom |
| Battery nearly full, Multi raising the frequency to throttle the Fronius | the script drains the battery a little so the PV keeps producing | solar tracking, probing Fronius |
| Kettle, oven, cloud | current cut at once, then back up slowly | protective cut, then recovery hold |

**Drain target** (battery discharge mode A aims for): 400 W from 98 % SOC, 700 W from 99 %, 1000 W from 99.5 %. Add 400 W while the AC frequency is at or above 50.25 Hz. Below 98 % with the frequency raised it is 800 W.

**The car's shortfall.** The Kona draws a little less than the pilot offers (about 1 A at 16 A, 2 A at 8 A). Modes B, C and D send exactly the current they show. Mode A measures the shortfall while the car charges steadily and asks for that much more, so the car really takes the surplus. The learned value is never more than 2.5 A, never raises the current above the measured inverter or battery headroom, and shows in HA as **Car shortfall**.

### Control loop

- Every **0.25 s**: read Victron data and protect. Reductions go to the gateway immediately, with 0.5 A extra margin.
- After a protective cut: **6 s** hold, then up again.
- Increases: at most every **3 s**, 2 A per step, or 1 A when the Multi is above 85 % of 4.7 kW.
- Changes smaller than 0.2 A are not sent.
- Every **1 s**: read the SDM120 and send one command to the gateway (current, measured power, meter kWh counter). The reply is the wallbox status.

### Starting and stopping

- **Start:** needs a car, a mode other than E, Charging enabled on, the gateway page switch on, SOC above the mode's minimum, and room for at least 8 A. A restart is always at 8 A, then the loop climbs.
- **Gentle stop** (SOC floor, mode E, HA switch off, not enough power): the current drops to 6 A so the car ends the session itself. Once the SDM120 reads under 300 W (after at least 3 s, at most 12 s), the script tells the gateway to pause and the pilot switches off. The contactor then opens with nothing flowing.
- **Immediate stop**, without the taper: wallbox errors and the gateway page switch.
- **Anti-flapping:** stays at least 30 s at the minimum before pausing for lack of power, then waits 90 s before restarting.

---

## Safety

- **Heartbeat.** The command the script sends every second is the gateway's heartbeat. If it stops for 180 s (gateway setting *Controller watchdog*), the gateway stops charging. The gateway's *Current when controller silent* must stay **0** on an off-grid system.
- **Stopping the service pauses the car** before the script exits.
- **Victron data older than 15 s:** the script holds 8 A until the data comes back.
- **Meter not answering:** the script estimates the power from the current it offers and carries on. It retries the meter after 2, 4, 8 and 16 s, then every 30 s.
- **The one gap:** if the gateway itself is dead, or its serial link to the board is broken, the wallbox keeps its last current and nothing can stop it over the network. The Multi's own overload protection is then the last line.
- **The gateway has no login.** Keep it on the home network and never port-forward port 80.
- **Never upload `config.json`** to GitHub. It holds the MQTT password. `evse_manager.py` itself contains no password.

---

## Requirements

- Venus OS on a Raspberry Pi, with root SSH access (and WinSCP or similar for copying files).
- Python 3 with `dbus` and `gi` (both ship with Venus OS) and `paho-mqtt` (installed by the setup block below).
- WT32-ETH01 gateway with firmware 2.0.x in front of the EVRACING EVSE-WB. Script 7.x cannot talk to gateway 1.0.1, and script 6.0 cannot talk to gateway 2.x.
- Eastron SDM120 on a USB-RS485 adapter, Modbus address 2, 9600 baud. It is optional, but mode A regulates on it.
- An MQTT broker (Mosquitto in Home Assistant).

---

## Upgrading an existing install

Do it while the car is **not charging**.

1. SSH to the Pi (`ssh root@192.168.31.16`) and back up the running script:
   ```sh
   cp /data/evse_manager/evse_manager.py /data/evse_manager/evse_manager.py.bak
   ```
2. With WinSCP, copy `evse_manager.py` from this repository to `/data/evse_manager/evse_manager.py` (overwrite). Leave `config.json` alone.
3. Restart and watch the log:
   ```sh
   svc -t /service/evse_manager
   tail -f /data/evse_manager/evse_manager.log
   ```
   After about 20 s (the service waits 15 s before starting the script) you should see `evse_manager 7.1.1 started (...)`. Ctrl+C stops watching; the service keeps running.

**Roll back:**
```sh
cp /data/evse_manager/evse_manager.py.bak /data/evse_manager/evse_manager.py && svc -t /service/evse_manager
```

---

## Fresh install (new Pi or new SD card)

**1. Create the folder.** SSH to the Pi and run:
```sh
mkdir -p /data/evse_manager
```

**2. Copy the script.** With WinSCP, copy `evse_manager.py` from this repository into `/data/evse_manager/`.

**3. Set up the service.** Paste this whole block into the SSH window. It checks the script, installs `paho-mqtt` if it is missing, creates `config.json` (only if there is none), writes the service files and makes the service start at every boot. It does not start the service yet.
```sh
sh <<'EOF'
D=/data/evse_manager
[ -f $D/evse_manager.py ] || { echo "ERROR: copy evse_manager.py to $D first"; exit 1; }
python3 -m py_compile $D/evse_manager.py || { echo "ERROR: evse_manager.py is damaged"; exit 1; }
grep -m1 '^VERSION' $D/evse_manager.py

if ! python3 -c "import paho.mqtt.client" 2>/dev/null; then
    echo ">>> installing paho-mqtt"
    opkg update </dev/null && opkg install python3-pip </dev/null
    pip3 install paho-mqtt </dev/null || pip3 install --break-system-packages paho-mqtt </dev/null
fi
python3 -c "import paho.mqtt.client" || { echo "ERROR: paho-mqtt is missing"; exit 1; }

if [ ! -f $D/config.json ]; then
cat > $D/config.json <<'CFG'
{
  "EVSE_IP": "192.168.31.125",
  "MQTT_HOST": "192.168.31.43",
  "MQTT_PORT": 1883,
  "MQTT_USER": "CHANGE-ME",
  "MQTT_PASS": "CHANGE-ME",
  "ABS_MIN_A": 8,
  "PILOT_CAL_A": 0
}
CFG
    echo ">>> config.json created - put your MQTT user and password in it"
else
    echo ">>> config.json already there - left as it is"
fi

mkdir -p $D/service/log /var/log/evse_manager
printf '#\041/bin/sh\nexec 2>&1\nsleep 15\nexec python3 /data/evse_manager/evse_manager.py\n' > $D/service/run
printf '#\041/bin/sh\nexec multilog t s250000 n5 /var/log/evse_manager\n' > $D/service/log/run
chmod +x $D/service/run $D/service/log/run

[ -f /data/rc.local ] || printf '#\041/bin/bash\n' > /data/rc.local
grep -q '/service/evse_manager' /data/rc.local || \
    echo 'ln -sfn /data/evse_manager/service /service/evse_manager' >> /data/rc.local
chmod +x /data/rc.local
echo ">>> done - now edit config.json, then start the service"
EOF
```

**4. Put in your MQTT login.** Replace both `CHANGE-ME` in `config.json`:
```sh
nano /data/evse_manager/config.json
```
Ctrl+X, then Y, then Enter saves.

**5. Start the service and check it:**
```sh
ln -sfn /data/evse_manager/service /service/evse_manager
sleep 25; svstat /service/evse_manager; tail -n 15 /data/evse_manager/evse_manager.log
```
`svstat` should say `up`. The run file waits 15 s before starting the script, so the first log lines appear after that.

**6. Meter adapter and reboot.** Set up the meter's USB adapter (next section), then reboot the Pi once to check that everything comes back by itself.

---

## The SDM120 meter and Venus OS

The script reads the SDM120 itself (Modbus RTU, function 04, address 2, 9600 baud) through the adapter at `METER_DEVICE`. **Venus OS must leave that adapter alone.** Venus's serial-starter claims USB serial adapters it recognises and starts its own services on them. Two processes on the same port corrupt each other's reads. Venus's own rules mistake:

- a **CH340** adapter (USB id `1a86:7523`) for a Zigbee converter (`cgwacs`),
- a generic **FTDI FT232R** adapter for a Victron RS485 cable (`rs485`).

The fix is a udev rule that runs *after* Venus's own `serial-starter.rules` (so its file name starts with `zz-`) and marks only that adapter as `ignore`.

**CH340 adapter (the current setup).** Paste this whole block:
```sh
sh <<'EOF'
RULE=/etc/udev/rules.d/zz-evse-meter-exclude.rules
cat > "$RULE" <<'R'
# Keep Venus serial-starter off the CH340 USB-RS485 adapter of the SDM120 meter.
# Runs after serial-starter.rules, so it wins. FTDI VE.Direct cables are not matched.
ACTION=="add", ENV{ID_BUS}=="usb", ENV{ID_VENDOR_ID}=="1a86", ENV{ID_MODEL_ID}=="7523", ENV{VE_SERVICE}="ignore"
R
mkdir -p /data/evse_manager
cp "$RULE" /data/evse_manager/zz-evse-meter-exclude.rules
[ -f /data/rc.local ] || printf '#\041/bin/bash\n' > /data/rc.local
grep -q 'zz-evse-meter-exclude' /data/rc.local || \
    echo 'cp /data/evse_manager/zz-evse-meter-exclude.rules /etc/udev/rules.d/ 2>/dev/null' >> /data/rc.local
chmod +x /data/rc.local
echo ">>> rule installed - reboot the Pi (or unplug and replug the adapter) to apply it"
EOF
```

**FTDI adapter instead.** Do **not** match FTDI in general (vendor `0403`): the VE.Direct cables to the MPPT and SmartShunt are FTDI too. Match that one adapter's serial number:
1. Plug it in and find its serial number and by-id path:
   ```sh
   ls -l /dev/serial/by-id/
   for d in /dev/ttyUSB*; do echo "$d"; udevadm info -q property -n $d | grep -E '^(ID_MODEL|ID_SERIAL_SHORT|VE_SERVICE)='; done
   ```
2. In the block above, replace the `ACTION==...` line with the one below, putting in your serial number, then paste the block:
   ```
   ACTION=="add", ENV{ID_BUS}=="usb", ENV{ID_SERIAL_SHORT}=="YOUR-SERIAL", ENV{VE_SERVICE}="ignore"
   ```
3. Set `METER_DEVICE` in `config.json` to the adapter's `/dev/serial/by-id/...` path, then reboot.

**Check that it worked** (after the reboot). For an FTDI adapter, put its own `/dev/serial/by-id/...` path in the first line.
```sh
M=$(readlink -f /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0); echo "meter port: $M"
udevadm info -q property -n "$M" | grep VE_SERVICE
ls /service | grep "${M##*/}" || echo "no Venus service on the meter port - good"
```
The second line must print `VE_SERVICE=ignore`, and the last must say there is no Venus service on the meter port. Services on the VE.Direct cables' ports are normal. HA *Supply voltage* should show the real mains voltage, for example 228.7 V.

> **Don't apply the rule with `udevadm trigger`.** On 20 Sep 2026 that made Venus re-detect every USB serial port, and the SmartShunt and MPPT dropped out until a reboot. Reboot, or unplug and replug the adapter, instead.

**After a Venus firmware update** the rule is copied back by `/data/rc.local` near the end of the first boot, so it only takes effect from the next boot. Reboot once more after every firmware update.

---

## Settings: `config.json`

`/data/evse_manager/config.json` overrides the defaults at the top of `evse_manager.py`. It is read only when the script starts, so restart the service after a change. Only names on the script's own list are accepted. A misspelt name shows up in the log as `ignored unknown config keys: ...`. Keys from versions up to 6.0 (`TRANSPORT`, `EVSE_WATCHDOG_S`, `METER_SOURCE` and others) are ignored and listed once.

Example:
```json
{
  "EVSE_IP": "192.168.31.125",
  "MQTT_HOST": "192.168.31.43",
  "MQTT_PORT": 1883,
  "MQTT_USER": "your-mqtt-user",
  "MQTT_PASS": "your-mqtt-password",
  "ABS_MIN_A": 8,
  "PILOT_CAL_A": 0
}
```

| Key | Default | Meaning |
|---|---|---|
| `EVSE_IP` | `192.168.31.125` | gateway address |
| `MQTT_HOST`, `MQTT_PORT` | `192.168.31.43`, `1883` | MQTT broker |
| `MQTT_USER`, `MQTT_PASS` | empty | MQTT login |
| `ABS_MIN_A` | `8` | lowest current offered; below it the car is paused instead |
| `ABS_MAX_A` | `32` | hardware and breaker ceiling |
| `INV_MAX_W` | `5000` | continuous Multi RS output allowed, W |
| `INV_RESERVE_W` | `300` | always kept free for load steps, W |
| `BATT_HARD_MAX_W` | `5000` | battery discharge ceiling in any mode, W |
| `SOC_HYSTERESIS` | `3` | resume at the mode's minimum SOC plus this, % |
| `A_SOC_DRAIN_START` | `98` | SOC where mode A starts draining the battery, % |
| `A_DRAIN_BASE_W`, `A_DRAIN_W_HI`, `A_DRAIN_W_MAX` | `400`, `700`, `1000` | drain targets, W |
| `A_DRAIN_SOC_HI`, `A_DRAIN_SOC_MAX` | `99`, `99.5` | SOC where the higher drain targets apply, % |
| `A_GAP_START_A`, `A_GAP_MAX_A` | `1.0`, `2.5` | car shortfall assumed before it is measured, and its upper limit, A |
| `METER_DEVICE` | `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` | meter adapter; `""` means no meter |
| `METER_BAUD`, `METER_UNIT` | `9600`, `2` | SDM120 serial speed and Modbus address |
| `METER_TARE` | `auto` | subtract the wallbox's own standby draw: `auto`, a number in W, or `off` |
| `STOP_TAPER_A`, `STOP_POWER_W`, `STOP_MAX_S` | `6`, `300`, `12` | gentle stop: taper current, "car has let go" power, give-up time |
| `MIN_DWELL_S`, `REPAUSE_HOLD_S` | `30`, `90` | anti-flapping timers, s |
| `PILOT_CAL_A` | `0` | fixed offset on every current. Keep it at 0: mode A learns the shortfall instead |

**Change one value** (keeps the old file as `config.json.bak` and restarts the script). This example keeps 400 W free instead of 300 W. For another setting, change only the name in `c["..."]` and the number after `=`. Paste it as a single line:
```sh
python3 -c 'import json,os,shutil;p="/data/evse_manager/config.json";c=json.load(open(p));shutil.copy2(p,p+".bak");c["INV_RESERVE_W"]=400;f=open(p+".tmp","w");json.dump(c,f,indent=2);f.close();os.replace(p+".tmp",p);print("OK - saved, old file kept as config.json.bak")' && svc -t /service/evse_manager && echo "Service restarted."
```
**Undo the last change:**
```sh
cp /data/evse_manager/config.json.bak /data/evse_manager/config.json && svc -t /service/evse_manager
```
**Which keys did the script take?**
```sh
grep "config.json" /data/evse_manager/evse_manager.log | tail -n 3
```

---

## Home Assistant and MQTT

The device **Wallbox** appears by MQTT discovery (prefix `homeassistant`, unique IDs `victron_evse_...`).

- **Controls:** Charging mode (select), Charging enabled (switch), Manual current (number, slider, used in mode D).
- **Sensors:** Vehicle connected, Charging, Charging power, Charging current, Allowed current, Session energy, Total energy (the SDM120's own counter, usable in the HA Energy dashboard), EVSE state, Limited by.
- **Diagnostics:** Car shortfall, Wallbox link problem, Energy meter problem, Supply voltage, Wallbox temperature, Wallbox uptime, Wallbox restarts, Wallbox unreachable for, Multi RS AC output, Battery power, Battery SOC, Fronius power, MPPT power, Battery drain target, AC out frequency.

The *Deadman remaining* sensor from 6.0 no longer exists (the watchdog now lives on the gateway). Remove it from older dashboards.

| Topic | Direction | Payload |
|---|---|---|
| `evse/mode/set` | to the script | mode label, or just the letter `A`…`E` |
| `evse/enable/set` | to the script | `ON` / `OFF` |
| `evse/manual_current/set` | to the script | whole amps, 8–32 |
| `evse/mode/state`, `evse/enable/state`, `evse/manual_current/state` | from the script | current values (retained) |
| `evse/availability` | from the script | `online` / `offline` (retained, `offline` is the last will) |
| `evse/state` | from the script | JSON once a second, everything the sensors show |

Example, switch to mode A from any MQTT client:
```sh
mosquitto_pub -h 192.168.31.43 -u <user> -P <password> -t evse/mode/set -m A
```

Other systems should control charging through MQTT, so the solar and battery protection stays in charge. Don't send commands to the gateway's `/api/set` while the script runs: it sends its own every second and the two would fight.

---

## Command line

```sh
python3 /data/evse_manager/evse_manager.py --doctor   # duplicate instances, gateway, meter, config
python3 /data/evse_manager/evse_manager.py --probe    # wallbox status from the gateway, once
python3 /data/evse_manager/evse_manager.py --dump     # every Victron D-Bus path this Venus OS has
```
Add `-v` for more detail in the log.

---

## Files on the Pi

All in `/data/evse_manager/`, which survives Venus OS updates.

| File | What it is |
|---|---|
| `evse_manager.py` | the script and its built-in defaults |
| `config.json` | your settings (contains the MQTT password, never upload it) |
| `state.json` | mode, on/off, manual current and the learned car shortfall, so they survive restarts |
| `evse_manager.log` | start-up, decisions, a status line every 60 s. Rotates at 1 MB, keeps 3 older files |
| `incidents.log` | link losses, gateway restarts, contactor opened under load, overdraw. Never rotated |
| `instance.lock` | stops a second copy of the script from running |
| `service/` | the daemontools service, linked to `/service/evse_manager` at boot by `/data/rc.local` |
| `zz-evse-meter-exclude.rules` | backup of the meter adapter rule, copied back after firmware updates |

---

## Everyday commands

```sh
tail -f /data/evse_manager/evse_manager.log                               # watch the log (Ctrl+C stops watching)
grep -E "WARNING|ERROR" /data/evse_manager/evse_manager.log | tail -n 30  # only warnings and errors
tail -n 20 /data/evse_manager/incidents.log                               # incident record
svstat /service/evse_manager                                              # is it running?
wget -qO- http://192.168.31.125/api/status                                # wallbox status as the gateway sees it
wget -qO- http://192.168.31.125/api/log                                   # gateway event log
grep -E "tapering|released|under load|pause charging" /data/evse_manager/evse_manager.log | tail -n 12   # how did the last stops end?
```

Commands that change something:
```sh
svc -t /service/evse_manager    # restart the script (the car pauses briefly)
svc -d /service/evse_manager    # stop it (the car pauses and stays paused)
svc -u /service/evse_manager    # start it again
```

A clean stop reads `car released after ... opening contactor cleanly` in the log.

---

## When something goes wrong

| Situation | What happens | Back to normal |
|---|---|---|
| Script stopped (`svc -d`, `svc -t`) | the script pauses the car on its way out | when the script starts again |
| Script crashes, Pi reboots | charging continues at the last current for up to 180 s, then the gateway stops | automatically, when the script is back |
| Victron data stale, script alive | holds 8 A, HA *Limited by: no D-Bus data* | automatically |
| SDM120 not answering, USB stick unplugged | power estimated from the offered current, HA *Energy meter problem* | automatically, the port is reopened |
| Home Assistant or MQTT down | nothing changes, charging logic carries on in the last mode | automatically |
| Gateway reboot (update, power blip) | the board keeps the last current, the gateway waits up to 15 s for the script | automatically |
| Network fails between Pi and gateway | the gateway stops after 180 s. After an outage over 30 s the script restarts gently from 8 A | automatically |
| Wallbox fault (pilot, diode, RCD) | the board opens the contactor, the script stops at once | after the fault clears and a 60 s wait. Repeated faults double the wait, up to 30 min. Unplugging the car clears it |
| Gateway page switch off | stops whatever the script or HA want | switch it back on, the script restarts at 8 A |
| HA *Supply voltage* stuck or *Energy meter problem* on | probably Venus has claimed the meter's USB adapter | see [The SDM120 meter and Venus OS](#the-sdm120-meter-and-venus-os) |

---

## Version history

| Date | Version | What changed |
|---|---|---|
| until 27 Sep 2026 | script up to 6.0 | Drove an ESP32-EVSE (dzurikmiroslav) wallbox over REST and Modbus TCP, later through gateway 1.0.1, which imitated that API. |
| 27 Sep 2026 | script 7.0, gateway 2.0.0 | One small JSON API (`POST /api/set` once a second, status in the reply). Pause via board register 2005 bit 14. SDM120 power forwarded to the gateway page. Mode A fix (32 A cap, drain when frequency-shifted), gateway page switch respected. Growing wait after faults, at most 30 EEPROM writes per hour, verified stops, meter reopened after a USB drop. |
| 30 Sep 2026 | gateway 2.0.1, config | Gateway page shows the requested current and "measured by SDM120". `PILOT_CAL_A` set to 0: no more fixed +2 A. |
| 2 Oct 2026 | gateway 2.0.2 | Total energy counted continuously while charging. Shows the SDM120's own counter. |
| 2 Oct 2026 | script 7.1 | Mode A learns the car's shortfall and allows for it, never beyond the measured headroom. SDM120 counter read for HA *Total energy* and the gateway page. New HA sensor *Car shortfall*. |
| 2 Oct 2026 | script 7.1.1 | Fixed: a stop that was called off, or fell back into the 30 s hold, left stale state, so the next stop skipped the taper and switched off under load. The bug dated back to 6.0. |
