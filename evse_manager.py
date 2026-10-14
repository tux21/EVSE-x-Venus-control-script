#!/usr/bin/env python3
# -*- coding: ascii -*-
"""
evse_manager.py  --  solar-following EV charge controller for Venus OS

Off-grid setup this is written for:
  * Victron Multi RS 6000 (48V) - inverter + 2 internal MPPTs
  * Victron SmartSolar 100/20   - external MPPT
  * Fronius Primo 3.6-1         - AC-coupled on the Multi RS AC-OUT (microgrid)
  * EVRACING EVSE-WB wallbox    - behind a WT32-ETH01 gateway, 1-phase 6..32 A
  * Eastron SDM120 meter        - USB-RS485 stick on this Pi, measures the car
  * Home Assistant              - MQTT autodiscovery

How it talks to the wallbox
---------------------------
One small HTTP request to the gateway, once a second:

    POST http://<EVSE_IP>/api/set   {"charge": true, "amps": 10.0, "power": 1840,
                                     "energy": 1234567}

The reply is the wallbox status (car connected, charging, faults ...). The same
request is the heartbeat: if it stops arriving for longer than the watchdog set
on the gateway's own settings page (180 s), the gateway stops charging by
itself. "power" is the SDM120 reading, so the gateway page shows real watts,
and "energy" is the SDM120's own kWh counter (gateway 2.0.2 shows it).

Control philosophy
------------------
Everything is CLOSED LOOP on measured values. The script never assumes how much
the Fronius "should" be making - it watches what the Multi RS is actually
pushing out of its AC-OUT terminals and what the battery is actually doing, and
moves the wallbox current until those numbers sit where they should.

Two cadences:
  * fast (LOOP_S, default 0.25 s) - protection. Reductions are applied instantly
    and without rate limit.
  * slow (UP_SETTLE_S, default 3 s) - increases only, in small steps, so the car
    has time to follow and so we probe carefully into Fronius territory.

Mode A solves the "battery hits 100 %, Victron frequency-shifts the Fronius to
sleep" problem by regulating BATTERY POWER to a small negative (discharge)
setpoint instead of computing a solar surplus. See mode_a_target().

Run `python3 evse_manager.py --dump` to print every D-Bus path this Venus OS
actually exposes, `--probe` to print the wallbox status once, and `--doctor`
to check for duplicate instances, the gateway and the meter.

Standard library only, plus paho-mqtt (and dbus/gi, which Venus OS ships).
"""

import argparse
import http.client
import json
import logging
import logging.handlers
import os
import signal
import struct
import sys
import threading
import time

# =============================================================================
#  CONFIGURATION  -  everything you may want to touch lives in this block
#  (your own values belong in /data/evse_manager/config.json, see CONFIG_FILE)
# =============================================================================

# ---- wallbox: the WT32-ETH01 gateway in front of the EVRACING board ------------
EVSE_IP = "192.168.31.125"
EVSE_HTTP_PORT = 80
EVSE_TIMEOUT_S = 1.5
# How often the gateway is asked for the wallbox status. Every one of these
# requests also carries the current command and is the gateway's heartbeat.
# Load-shedding does not wait for it: it reacts to Victron D-Bus data at LOOP_S
# and a changed command is sent to the gateway immediately.
EVSE_POLL_S = 1.0

# ---- Home Assistant MQTT broker ----------------------------------------------
MQTT_HOST = "192.168.31.43"
MQTT_PORT = 1883
MQTT_USER = ""              # set "MQTT_USER" in config.json
MQTT_PASS = ""              # set "MQTT_PASS" in config.json
MQTT_PREFIX = "evse"        # topic root
HA_PREFIX = "homeassistant" # HA discovery prefix

# ---- system limits -----------------------------------------------------------
# Continuous AC output you are willing to pull through the Multi RS.
# Nameplate is 6000 VA; 5000 W is a sane continuous figure for a warm cabinet.
INV_MAX_W = 5000.0
# Always kept free so a load step (water heater!) does not instantly overload.
INV_RESERVE_W = 300.0
# Absolute battery discharge ceiling, any mode. Set to your BMS/cable rating.
BATT_HARD_MAX_W = 5000.0
# DC-coupled PV is worth slightly less on the AC side (inverter efficiency).
DC_TO_AC_EFF = 0.94
# Assumed AC voltage if the Multi does not report one.
NOMINAL_VAC = 230.0

# ---- charging current envelope ----------------------------------------------
# Lowest current worth offering the car. IEC 61851 allows 6 A, but many EVs
# will not charge that low: a Hyundai Kona drops the session below ~8 A and
# then sits there drawing a few hundred watts of onboard-charger overhead for
# no useful energy. Below this figure we PAUSE instead of offering less, which
# is strictly better than paying that overhead. Override in config.json.
ABS_MIN_A = 8.0
ABS_MAX_A = 32.0            # hardware / breaker ceiling
RESUME_A = ABS_MIN_A        # current we restart at after a pause

# --- stopping gracefully ------------------------------------------------------
# Removing the pilot signal does not reliably stop a car quickly. J1772 gives
# the vehicle up to 3 s, but a Kona keeps asserting state C well past that, so
# the EVSE would open its contactor under load.
#
# So rather than ordering the car to stop, we make it decide to stop: taper the
# offered current BELOW the car's own minimum. A Kona abandons the session under
# ~8 A, opens its contactor and drops to state B by itself. Only once the meter
# confirms the current has actually gone do we tell the gateway to pause - by
# which point the contactor opens with nothing flowing through it.
EVSE_HW_MIN_A = 6.0         # IEC 61851 floor; the lowest the hardware may offer
STOP_TAPER_A = 6.0          # current used to coax the car into letting go
STOP_POWER_W = 300.0        # below this the car has clearly stopped drawing
STOP_MAX_S = 12.0           # give up waiting and open anyway
STOP_RAMP_S = 3.0           # minimum settle time even if power reads zero

# Anti-chatter around the minimum. Without these, a thin cloud drags the solar
# target just under ABS_MIN_A, we stop the car, the sun returns a few seconds
# later and we start it again - repeatedly, which no EV enjoys.
MIN_DWELL_S = 30.0          # keep charging at the minimum this long before
                            # accepting there really is not enough power
REPAUSE_HOLD_S = 90.0       # after such a pause, wait this long before
                            # restarting the session

# --- the car's shortfall -------------------------------------------------------
# The Kona draws a little less than the pilot offers, and not always the same:
# measured 2 A short at 8 A and 1.8 A at 18 A in September, about 1 A at 16 A in
# October. So nothing is added on the way to the wallbox any more: Home
# Assistant, the gateway page and the pilot all show the same current, and the
# SDM120 shows what the car really takes.
#
# Only mode A needs to know the shortfall, because it works out how many WATTS
# the car may take. It learns the shortfall while the car charges steadily
# (offered minus measured amps, smoothed) and asks for that much more, so the
# car really takes the surplus. It is never used for the inverter or battery
# headroom, and modes B, C and D send exactly the current they show.
A_GAP_START_A = 1.0         # used until the shortfall has been measured
A_GAP_MAX_A = 2.5           # never assume more than this
GAP_SETTLE_S = 8.0          # the offer must be steady this long before a sample
GAP_START_S = 60.0          # no samples in the first minute of a charge

# Fixed offset added to every current sent to the wallbox (7.0 used 2 A). Kept
# only for compatibility; with the learned shortfall above, leave it at 0.
PILOT_CAL_A = 0.0

# ---- energy meter: Eastron SDM120 on a USB-RS485 stick plugged into this Pi --
# The meter is the only real measurement of the car's power - the EVRACING board
# has none - and mode A regulates on it. We read it ourselves (Modbus RTU, FC04),
# which works for generic CH340/PL2303 sticks that Venus would not claim. If the
# meter cannot be read, the power is estimated from the current we offer and
# charging carries on. Set METER_DEVICE to "" if there is no meter at all.
METER_DEVICE = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
METER_BAUD = 9600
METER_UNIT = 2              # SDM120 Modbus address (its default is 1)
METER_TIMEOUT_S = 1.0
# The meter's own kWh counter (total active energy) is read this often. It goes
# to Home Assistant as "Total energy" and to the gateway page.
METER_ENERGY_EVERY_S = 10.0

# The meter also sees the wallbox's own standby draw (gateway, EVSE board,
# contactor coil) - a few watts that would make "charging power" sit at 8 W
# instead of 0 when idle. "auto" learns it whenever the car is not charging
# and subtracts it; a number sets a fixed offset in watts; "off" disables it.
METER_TARE = "auto"
METER_TARE_MAX_W = 120.0    # anything above this is not standby, do not learn it

# ---- mode definitions --------------------------------------------------------
# i_max     : hard current cap for the mode
# i_floor   : current the mode will hold even with no sun (0 = solar only)
# soc_min   : stop charging below this battery SOC
# batt_ev_w : how much of the EV's power may come from the battery
MODES = {
    # Mode A is "run the car on sun, with an 8 A floor". The battery does not
    # top the car up in addition to the sun - it only fills the gap when the
    # sun cannot reach the floor, and it deliberately discharges a little when
    # nearly full so the Fronius is not frequency-shifted into idleness.
    # batt_ev_w is therefore 0 here: the allowance comes from drain_target().
    "A": dict(i_max=32.0, i_floor=8.0, soc_min=30.0, batt_ev_w=0.0),
    "B": dict(i_max=25.0, i_floor=6.0, soc_min=50.0, batt_ev_w=5000.0),
    "C": dict(i_max=32.0, i_floor=6.0, soc_min=50.0, batt_ev_w=5000.0),
    "D": dict(i_max=32.0, i_floor=0.0, soc_min=30.0, batt_ev_w=BATT_HARD_MAX_W),
    "E": dict(i_max=0.0, i_floor=0.0, soc_min=101.0, batt_ev_w=0.0),
}
MODE_LABELS = {
    "A": "A - Automatic (solar + 8A battery)",
    "B": "B - Fast (25A, 5kW battery)",
    "C": "C - Super fast (32A, 5kW battery)",
    "D": "D - Manual current",
    "E": "E - Charging disabled",
}
LABEL_TO_MODE = dict((v, k) for k, v in MODE_LABELS.items())

SOC_HYSTERESIS = 3.0        # resume charging at soc_min + this

# ---- mode A: battery stays out of it until the battery is nearly full --------
# Below this SOC the battery contributes NOTHING to the car beyond what the 8 A
# floor forces - unless the Multi is already frequency-shifting (see below).
A_SOC_DRAIN_START = 98.0

# Victron raises AC-OUT frequency to throttle the Fronius once the battery is
# full. Holding a small permanent discharge keeps the battery out of float, so
# the Fronius keeps producing and the car gets that energy instead.
A_DRAIN_BASE_W = 400.0      # at A_SOC_DRAIN_START
A_DRAIN_SOC_HI = 99.0       # higher still, pull harder
A_DRAIN_W_HI = 700.0
A_DRAIN_SOC_MAX = 99.5      # essentially full - the Fronius is being throttled
A_DRAIN_W_MAX = 1000.0
A_DRAIN_FREQ_BONUS_W = 400.0  # extra drain while frequency shift is detected
A_GAIN = 0.6                # feedback gain on the drain error (0..1)

FREQ_SHIFT_HZ = 50.25       # above this the Multi is curtailing the Fronius

# ---- loop timing -------------------------------------------------------------
LOOP_S = 0.25               # protection / measurement cadence
UP_SETTLE_S = 3.0           # minimum interval between current increases
UP_STEP_A = 2.0             # normal increase step
UP_STEP_PROBE_A = 1.0       # increase step while probing into Fronius territory
RECOVERY_HOLD_S = 6.0       # no increases for this long after a protective cut
CUT_MARGIN_A = 0.5          # extra bite taken on every protective cut
# Deadband around the target. Nothing moves while the error is inside it, which
# stops needless commands but also leaves a steady state offset of about
# deadband * 230 W. 0.2 A -> up to ~46 W.
WRITE_DEADBAND_A = 0.2
DOWN_DEADBAND_A = 0.2
DBUS_STALE_S = 15.0         # no D-Bus update for this long -> fall back to safe
DBUS_REFRESH_S = 5.0        # periodic re-read of tracked paths
# After the gateway was unreachable for longer than this, start again gently
# from RESUME_A instead of jumping straight back to the old current.
RELINK_GENTLE_S = 30.0

# ---- files -------------------------------------------------------------------
BASE_DIR = "/data/evse_manager"
STATE_FILE = os.path.join(BASE_DIR, "state.json")
LOG_FILE = os.path.join(BASE_DIR, "evse_manager.log")
# Your settings live HERE, not in this file. Reinstalling overwrites
# evse_manager.py wholesale; config.json is created once and never touched.
CONFIG_FILE = os.path.join(BASE_DIR, "config.json")
LOCK_FILE = os.path.join(BASE_DIR, "instance.lock")
# Append-only record of link losses, gateway restarts and contactor events,
# kept on disk so it survives everything.
INCIDENT_FILE = os.path.join(BASE_DIR, "incidents.log")

VERSION = "7.1.1"

# Settings that config.json may override. Anything not listed is ignored, so a
# typo in the config cannot quietly redefine part of the control logic.
OVERRIDABLE = (
    "MQTT_HOST", "MQTT_PORT", "MQTT_USER", "MQTT_PASS",
    "EVSE_IP", "EVSE_HTTP_PORT", "EVSE_TIMEOUT_S", "EVSE_POLL_S",
    "INV_MAX_W", "INV_RESERVE_W", "BATT_HARD_MAX_W",
    "A_DRAIN_BASE_W", "A_DRAIN_W_HI", "A_DRAIN_W_MAX",
    "A_DRAIN_SOC_HI", "A_DRAIN_SOC_MAX", "A_GAIN", "A_SOC_DRAIN_START",
    "LOOP_S", "UP_SETTLE_S",
    "ABS_MIN_A", "ABS_MAX_A", "MIN_DWELL_S", "REPAUSE_HOLD_S", "STOP_RAMP_S",
    "STOP_TAPER_A", "STOP_POWER_W", "STOP_MAX_S", "EVSE_HW_MIN_A",
    "PILOT_CAL_A", "A_GAP_START_A", "A_GAP_MAX_A",
    "METER_DEVICE", "METER_BAUD", "METER_UNIT", "METER_TARE",
    "METER_TARE_MAX_W",
    "SOC_HYSTERESIS",
)
# Keys that versions up to 6.0 understood (esp32-evse transports, dead-man
# timer, other meter paths). An old config.json may still contain them; they
# are ignored and listed once in the log.
RETIRED = (
    "TRANSPORT", "EVSE_PORT", "EVSE_UNIT", "EVSE_USER", "EVSE_PASS",
    "EVSE_WATCHDOG_S", "METER_SOURCE", "METER_ENABLED", "METER_HOST",
    "METER_PORT", "METER_FRAMING", "EVSE_FRAMING", "DIAG_MODBUS",
)


def record_incident(text):
    """
    Append a timestamped line to a file that nothing rotates away.
    """
    try:
        os.makedirs(BASE_DIR, exist_ok=True)
        with open(INCIDENT_FILE, "a") as handle:
            handle.write("%s  %s\n"
                         % (time.strftime("%Y-%m-%d %H:%M:%S"), text))
    except Exception:
        pass


def apply_min_current():
    """
    Propagate ABS_MIN_A into everything derived from it.

    MODES and RESUME_A are literals evaluated at import time, so a config
    override of ABS_MIN_A would otherwise leave mode B and C still holding a
    6 A floor. Run this after load_config().
    """
    global RESUME_A
    RESUME_A = ABS_MIN_A
    for name, cfg in MODES.items():
        # i_floor == 0 is the sentinel for "no floor" (modes D and E) and must
        # stay zero; every real floor is raised to at least the minimum.
        if cfg["i_floor"] > 0:
            cfg["i_floor"] = max(cfg["i_floor"], ABS_MIN_A)
        if cfg["i_max"] > 0:
            cfg["i_max"] = max(cfg["i_max"], ABS_MIN_A)


def load_config():
    """
    Apply /data/evse_manager/config.json over the defaults above.

    Returns (applied_keys, retired_keys, error). Must run before anything reads
    these values - i.e. before the Controller and the MQTT client are built.
    """
    try:
        with open(CONFIG_FILE, "r") as handle:
            data = json.load(handle)
    except IOError:
        return ([], [], None)
    except ValueError as exc:
        return ([], [], "config.json is not valid JSON: %s" % exc)
    if not isinstance(data, dict):
        return ([], [], "config.json must contain a JSON object")
    applied, retired, unknown = [], [], []
    scope = globals()
    for key, value in data.items():
        if key.startswith("_"):
            continue
        if key in OVERRIDABLE:
            scope[key] = value
            applied.append(key)
        elif key in RETIRED:
            retired.append(key)
        else:
            unknown.append(key)
    err = None
    if unknown:
        err = "ignored unknown config keys: %s" % ", ".join(sorted(unknown))
    return (applied, retired, err)


class LinkError(Exception):
    """The gateway or the meter could not be reached, or answered nonsense."""
    pass


# =============================================================================
#  Wallbox: WT32-ETH01 gateway, JSON over HTTP
# =============================================================================

class Gateway(object):
    """
    The wallbox, as the WT32-ETH01 gateway presents it. Two requests:

      status()                     GET  /api/status   read only
      command(charge, amps, power, energy)
                                   POST /api/set      command + heartbeat,
                                                      answers with the status

    A fresh TCP connection per request: the gateway closes every connection
    after its reply anyway, and on the LAN that costs a few milliseconds.
    """

    def __init__(self, host, port=80, timeout=1.5):
        self.host = host
        self.port = int(port)
        self.timeout = float(timeout)

    def _request(self, method, path, body=None):
        headers = {"Accept": "application/json"}
        payload = None
        if body is not None:
            payload = json.dumps(body)
            headers["Content-Type"] = "application/json"
        conn = http.client.HTTPConnection(self.host, self.port,
                                          timeout=self.timeout)
        try:
            conn.request(method, path, payload, headers)
            resp = conn.getresponse()
            raw = resp.read()
            status = resp.status
        except Exception as exc:
            raise LinkError("%s %s: %s" % (method, path, exc))
        finally:
            try:
                conn.close()
            except Exception:
                pass
        try:
            data = json.loads(raw.decode("utf-8", "ignore"))
        except ValueError:
            data = None
        if status != 200:
            detail = data.get("error") if isinstance(data, dict) else raw[:60]
            raise LinkError("%s %s -> HTTP %d (%s)" % (method, path, status,
                                                        detail))
        if not isinstance(data, dict):
            raise LinkError("%s %s: the answer is not a JSON object"
                            % (method, path))
        return data

    def status(self):
        return self.decode(self._request("GET", "/api/status"))

    def command(self, charge, amps, power=None, energy_wh=None):
        # Gateway 2.0.1 and older ignore "energy"; 2.0.2 shows it as the total.
        body = {
            "charge": bool(charge),
            "amps": round(max(EVSE_HW_MIN_A, min(ABS_MAX_A, float(amps))), 1),
            "power": None if power is None else int(round(power)),
            "energy": None if energy_wh is None else int(round(energy_wh)),
        }
        return self.decode(self._request("POST", "/api/set", body))

    @staticmethod
    def decode(js):
        """The gateway status, in the names the controller uses."""
        temp = js.get("temperature") or {}
        request = js.get("request") or {}
        session = js.get("session") or {}
        return {
            "link": bool(js.get("link")),        # gateway <-> EVSE board
            "state": str(js.get("state") or "?"),
            "connected": bool(js.get("connected")),
            "charging": bool(js.get("charging")),
            "errors": list(js.get("errors") or []),
            "reason": str(js.get("reason") or ""),
            "offered": float(js.get("offered") or 0.0),
            "current_set": float(request.get("amps") or 0.0),
            "enabled": bool(request.get("charge")),
            "switch": bool(js.get("switch", True)),
            "holdoff": int(js.get("holdoff") or 0),
            "session_wh": int(session.get("energy") or 0),
            "session_s": int(session.get("time") or 0),
            "total_wh": js.get("total_energy"),
            "uptime": js.get("uptime"),
            "temp_hi": temp.get("high"),
            "temp_lo": temp.get("low"),
            "fw": js.get("fw"),
        }


def format_errors(errors):
    if not errors:
        return ""
    if isinstance(errors, (list, tuple, set)):
        return ", ".join(str(e) for e in errors)
    return str(errors)


# =============================================================================
#  Energy meter: SDM120, Modbus RTU on a local serial port
# =============================================================================

def modbus_crc(data):
    """Modbus RTU CRC-16, returned little-endian as it goes on the wire."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return struct.pack("<H", crc)


def f32(regs, index):
    """Two big-endian registers -> IEEE754 float, as Eastron encodes them."""
    return struct.unpack(">f", struct.pack(">HH", regs[index],
                                           regs[index + 1]))[0]


class ModbusSerial(object):
    """
    Modbus RTU master on a local serial port, using termios from the standard
    library rather than pyserial - one less thing that can be missing or
    change on a Venus OS update.

    A missed or garbled reply is simply an error for that one read; the port
    is only closed and re-opened when the operating system reports a problem
    with it (stick unplugged, device gone).
    """

    REOPEN_S = 2.0

    def __init__(self, device, baud, unit, timeout):
        self.device = device
        self.baud = int(baud)
        self.unit = int(unit)
        self.timeout = float(timeout)
        self.fd = None
        self.reopen_at = 0.0

    def close(self):
        fd, self.fd = self.fd, None
        if fd is not None:
            try:
                os.close(fd)
            except OSError:
                pass

    def _lost(self, why):
        self.close()
        self.reopen_at = time.monotonic() + self.REOPEN_S
        return LinkError(why)

    def _ensure(self):
        if self.fd is not None:
            return
        if time.monotonic() < self.reopen_at:
            raise LinkError("%s not open yet" % self.device)
        import termios
        try:
            fd = os.open(self.device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except Exception as exc:
            raise self._lost("open %s: %s" % (self.device, exc))
        try:
            speed = getattr(termios, "B%d" % self.baud)
            attrs = termios.tcgetattr(fd)
            cc = list(attrs[6])
            cc[termios.VMIN] = 0
            cc[termios.VTIME] = 0
            # 8N1, raw, no flow control, ignore modem lines
            termios.tcsetattr(fd, termios.TCSANOW,
                              [0, 0, termios.CS8 | termios.CREAD |
                               termios.CLOCAL, 0, speed, speed, cc])
            termios.tcflush(fd, termios.TCIOFLUSH)
        except Exception as exc:
            os.close(fd)
            raise self._lost("configure %s: %s" % (self.device, exc))
        self.fd = fd

    def _read_exact(self, n):
        import select
        buf = b""
        deadline = time.monotonic() + self.timeout
        while len(buf) < n:
            left = deadline - time.monotonic()
            if left <= 0:
                raise LinkError("timeout after %d of %d bytes" % (len(buf), n))
            ready, _, _ = select.select([self.fd], [], [], left)
            if not ready:
                continue
            try:
                chunk = os.read(self.fd, n - len(buf))
            except BlockingIOError:
                continue
            if not chunk:
                raise self._lost("%s closed" % self.device)
            buf += chunk
        return buf

    def read_input(self, address, count):
        """FC04 - Eastron meters expose their measurements as input registers."""
        import termios
        self._ensure()
        frame = struct.pack(">BBHH", self.unit, 4, address, count)
        frame += modbus_crc(frame)
        try:
            termios.tcflush(self.fd, termios.TCIFLUSH)
            os.write(self.fd, frame)
            head = self._read_exact(3)
            unit, fc, third = head[0], head[1], head[2]
            tail = self._read_exact(2 if fc & 0x80 else third + 2)
        except LinkError:
            raise
        except (OSError, termios.error) as exc:
            # USB stick reset or unplugged: close, and reopen on a later read
            raise self._lost("%s: %s" % (self.device, exc))
        body = head + tail
        if modbus_crc(body[:-2]) != body[-2:]:
            raise LinkError("CRC mismatch")
        if unit != self.unit:
            raise LinkError("reply from unit %d, expected %d" % (unit, self.unit))
        if fc & 0x80:
            raise LinkError("modbus exception %d" % third)
        if third < count * 2:
            raise LinkError("short reply at %d" % address)
        return list(struct.unpack(">%dH" % count, body[3:3 + count * 2]))


# SDM120 input-register map (FC04), each value a big-endian IEEE754 float.
SDM_VOLTAGE = 0x0000
SDM_CURRENT = 0x0006
SDM_POWER = 0x000C
SDM_TOTAL_ENERGY = 0x0156   # total active energy, kWh (the meter's own counter)


class EnergyMeter(object):
    """
    Voltage, current and power of the wallbox feed from the SDM120.

    Kept deliberately separate from the wallbox link: if the meter goes away
    the power is estimated and charging carries on, because a measurement
    must never become a new way for the system to fail.
    """

    def __init__(self, log, bus):
        self.log = log
        self.bus = bus
        self.fails = 0
        self.announced = False
        self.skip_until = 0.0     # after failures, do not wait on a dead meter every poll
        self.energy_fails = 0

    def close(self):
        self.bus.close()

    def read(self):
        """Returns (volts, amps, watts) or None. Never raises."""
        now = time.monotonic()
        if now < self.skip_until:
            return None
        try:
            # One block covers 0x0000..0x000D: voltage, current and power.
            regs = self.bus.read_input(SDM_VOLTAGE, 14)
            volts = f32(regs, SDM_VOLTAGE)
            amps = f32(regs, SDM_CURRENT)
            watts = f32(regs, SDM_POWER)
        except Exception as exc:
            self.fails += 1
            if self.fails in (1, 20, 200):
                self.log.warning("energy meter unreachable: %s", exc)
            # Every attempt at a silent meter costs a full timeout of the control
            # loop, so back off: 2, 4, 8, 16, then every 30 s.
            self.skip_until = now + min(30.0, 2.0 ** min(self.fails, 5))
            return None
        for v in (volts, amps, watts):
            if v != v or abs(v) > 1e6:          # NaN or nonsense
                return None
        if self.fails or not self.announced:
            self.announced = True
            self.log.info("energy meter online: %.1f V %.2f A %.0f W",
                          volts, amps, watts)
        self.fails = 0
        return (volts, amps, watts)

    def read_energy(self):
        """The meter's total active energy counter in kWh, or None. Never raises."""
        if time.monotonic() < self.skip_until:
            return None
        try:
            kwh = f32(self.bus.read_input(SDM_TOTAL_ENERGY, 2), 0)
        except Exception as exc:
            self.energy_fails += 1
            if self.energy_fails in (1, 20, 200):
                self.log.warning("energy meter: total energy not readable: %s", exc)
            return None
        if kwh != kwh or kwh < 0 or kwh > 1e7:
            return None
        self.energy_fails = 0
        return kwh




# =============================================================================
#  Victron D-Bus monitor
# =============================================================================

TRACK_CLASSES = ("system", "multi", "vebus", "inverter", "acload")

# Paths we care about. Anything missing simply reads back as None and the
# controller falls back to a derived value.
WANT = {
    "system": (
        "/Dc/Battery/Soc",
        "/Dc/Battery/Power",
        "/Dc/Battery/Voltage",
        "/Dc/Battery/Current",
        "/Dc/Pv/Power",
        "/Ac/PvOnOutput/L1/Power",
        "/Ac/ConsumptionOnOutput/L1/Power",
        "/Ac/Consumption/L1/Power",
    ),
    "multi": (
        "/Ac/Out/L1/P",
        "/Ac/Out/L1/V",
        "/Ac/Out/L1/F",
        "/Dc/0/Power",
        "/Yield/Power",
        "/State",
    ),
}
WANT["acload"] = (
    "/Ac/L1/Power",
    "/Ac/L1/Current",
    "/Ac/L1/Voltage",
    "/Ac/Power",
)
WANT["vebus"] = WANT["multi"]
WANT["inverter"] = WANT["multi"]


class VeDbus(object):
    """
    Push-based monitor. Subscribes to ItemsChanged / PropertiesChanged so a
    changing load shows up here within milliseconds of Venus publishing it,
    instead of up to a full poll interval later.
    """

    def __init__(self, log):
        self.log = log
        self.lock = threading.Lock()
        self.values = {}      # (service, path) -> value
        self.owners = {}      # unique bus name -> well known name
        self.classes = {}     # well known name -> class ("system", "multi", ...)
        self.last_update = 0.0
        self.ok = False

        import dbus
        from dbus.mainloop.glib import DBusGMainLoop
        self.dbus = dbus
        DBusGMainLoop(set_as_default=True)
        self.bus = dbus.SystemBus()

        self.scan()

        self.bus.add_signal_receiver(
            self._on_items_changed,
            dbus_interface="com.victronenergy.BusItem",
            signal_name="ItemsChanged",
            path="/",
            sender_keyword="sender")
        self.bus.add_signal_receiver(
            self._on_props_changed,
            dbus_interface="com.victronenergy.BusItem",
            signal_name="PropertiesChanged",
            path_keyword="path",
            sender_keyword="sender")
        self.bus.add_signal_receiver(
            self._on_owner_changed,
            dbus_interface="org.freedesktop.DBus",
            signal_name="NameOwnerChanged")

    # ---- helpers ------------------------------------------------------------

    @staticmethod
    def _conv(value):
        # Victron publishes an empty array to mean "invalid / not available"
        if isinstance(value, (list, tuple)):
            return None
        try:
            return float(value)
        except (TypeError, ValueError):
            try:
                return str(value)
            except Exception:
                return None

    @staticmethod
    def _classof(name):
        parts = name.split(".")
        return parts[2] if len(parts) > 2 else ""

    def scan(self):
        try:
            names = [str(n) for n in self.bus.list_names()]
        except Exception as exc:
            self.log.error("dbus list_names failed: %s", exc)
            return
        for name in names:
            if not name.startswith("com.victronenergy."):
                continue
            cls = self._classof(name)
            if cls not in TRACK_CLASSES:
                continue
            self.classes[name] = cls
            try:
                self.owners[str(self.bus.get_name_owner(name))] = name
            except Exception:
                pass
            self._fetch_all(name, cls)
        self.ok = bool(self.classes)
        self.log.info("d-bus services tracked: %s", ", ".join(sorted(self.classes)))

    def _fetch_all(self, service, cls):
        wanted = WANT.get(cls)
        items = None
        try:
            obj = self.bus.get_object(service, "/", introspect=False)
            items = obj.GetItems(dbus_interface="com.victronenergy.BusItem")
        except Exception:
            items = None
        with self.lock:
            if items is not None:
                for path, props in items.items():
                    path = str(path)
                    if wanted is not None and path not in wanted:
                        continue
                    self.values[(service, path)] = self._conv(props.get("Value"))
            elif wanted:
                for path in wanted:
                    self.values[(service, path)] = self._read_one(service, path)
            self.last_update = time.time()

    def _read_one(self, service, path):
        try:
            obj = self.bus.get_object(service, path, introspect=False)
            return self._conv(obj.GetValue(dbus_interface="com.victronenergy.BusItem"))
        except Exception:
            return None

    def refresh(self):
        """Safety net re-read; also proves the bus is still alive."""
        for service, cls in list(self.classes.items()):
            for path in WANT.get(cls, ()):
                value = self._read_one(service, path)
                if value is not None:
                    with self.lock:
                        self.values[(service, path)] = value
        with self.lock:
            self.last_update = time.time()
        return True

    # ---- signal handlers ----------------------------------------------------

    def _service_of(self, sender):
        name = self.owners.get(str(sender))
        if name is None:
            try:
                for known, cls in self.classes.items():
                    if str(self.bus.get_name_owner(known)) == str(sender):
                        self.owners[str(sender)] = known
                        return known
            except Exception:
                return None
        return name

    def _on_items_changed(self, items, sender=None):
        service = self._service_of(sender)
        if service is None:
            return
        wanted = WANT.get(self.classes.get(service, ""), ())
        with self.lock:
            for path, props in items.items():
                path = str(path)
                if wanted and path not in wanted:
                    continue
                self.values[(service, path)] = self._conv(props.get("Value"))
            self.last_update = time.time()

    def _on_props_changed(self, changes, path=None, sender=None):
        service = self._service_of(sender)
        if service is None or path is None:
            return
        path = str(path)
        wanted = WANT.get(self.classes.get(service, ""), ())
        if wanted and path not in wanted:
            return
        with self.lock:
            self.values[(service, path)] = self._conv(changes.get("Value"))
            self.last_update = time.time()

    def _on_owner_changed(self, name, old, new):
        name = str(name)
        if not name.startswith("com.victronenergy."):
            return
        if self._classof(name) not in TRACK_CLASSES:
            return
        self.log.info("d-bus service changed: %s", name)
        self.scan()

    # ---- accessors ----------------------------------------------------------

    def get(self, cls, path, default=None):
        with self.lock:
            for service, klass in self.classes.items():
                if klass != cls:
                    continue
                value = self.values.get((service, path))
                if value is not None:
                    return value
        return default

    def get_any(self, classes, path, default=None):
        for cls in classes:
            value = self.get(cls, path)
            if value is not None:
                return value
        return default

    def stale(self):
        with self.lock:
            return (time.time() - self.last_update) > DBUS_STALE_S

    def dump(self):
        out = {}
        for service in sorted(self.classes):
            try:
                obj = self.bus.get_object(service, "/", introspect=False)
                items = obj.GetItems(dbus_interface="com.victronenergy.BusItem")
                out[service] = dict(
                    (str(p), self._conv(v.get("Value"))) for p, v in items.items())
            except Exception as exc:
                out[service] = {"__error__": str(exc)}
        return out


# =============================================================================
#  Home Assistant MQTT autodiscovery
# =============================================================================

DEVICE = {
    "identifiers": ["victron_evse_wallbox"],
    "name": "Wallbox",
    "manufacturer": "EVRACING",
    "model": "EVSE-WB + WT32-ETH01 gateway / Victron Multi RS 6000",
    "sw_version": VERSION,
}

T_STATE = MQTT_PREFIX + "/state"
T_AVAIL = MQTT_PREFIX + "/availability"
T_MODE = MQTT_PREFIX + "/mode"
T_MANUAL = MQTT_PREFIX + "/manual_current"
T_ENABLE = MQTT_PREFIX + "/enable"


# Entities earlier versions published. An empty retained config message
# removes them from Home Assistant.
RETIRED_ENTITIES = (("sensor", "watchdog"),)


def _entity(component, object_id, cfg):
    cfg = dict(cfg)
    cfg["unique_id"] = "victron_evse_" + object_id
    cfg["device"] = DEVICE
    cfg["availability_topic"] = T_AVAIL
    return ("%s/%s/victron_evse/%s/config" % (HA_PREFIX, component, object_id),
            json.dumps(cfg))


def discovery_payloads():
    tpl = lambda key: "{{ value_json.%s }}" % key
    out = [
        _entity("select", "mode", {
            "name": "Charging mode",
            "state_topic": T_MODE + "/state",
            "command_topic": T_MODE + "/set",
            "options": [MODE_LABELS[k] for k in ("A", "B", "C", "D", "E")],
            "icon": "mdi:ev-station",
        }),
        _entity("switch", "enable", {
            "name": "Charging enabled",
            "state_topic": T_ENABLE + "/state",
            "command_topic": T_ENABLE + "/set",
            "payload_on": "ON",
            "payload_off": "OFF",
            "icon": "mdi:power",
        }),
        _entity("number", "manual_current", {
            "name": "Manual current",
            "state_topic": T_MANUAL + "/state",
            "command_topic": T_MANUAL + "/set",
            "min": ABS_MIN_A,
            "max": ABS_MAX_A,
            "step": 1,
            "unit_of_measurement": "A",
            "mode": "slider",
            "icon": "mdi:current-ac",
        }),
        _entity("binary_sensor", "connected", {
            "name": "Vehicle connected",
            "state_topic": T_STATE,
            "value_template": "{{ 'ON' if value_json.connected else 'OFF' }}",
            "device_class": "plug",
        }),
        _entity("binary_sensor", "charging", {
            "name": "Charging",
            "state_topic": T_STATE,
            "value_template": "{{ 'ON' if value_json.charging else 'OFF' }}",
            "device_class": "battery_charging",
        }),
        _entity("sensor", "power", {
            "name": "Charging power",
            "state_topic": T_STATE,
            "value_template": tpl("power"),
            "unit_of_measurement": "W",
            "device_class": "power",
            "state_class": "measurement",
        }),
        _entity("sensor", "current", {
            "name": "Charging current",
            "state_topic": T_STATE,
            "value_template": tpl("current"),
            "unit_of_measurement": "A",
            "device_class": "current",
            "state_class": "measurement",
        }),
        _entity("sensor", "applied", {
            "name": "Allowed current",
            "state_topic": T_STATE,
            "value_template": tpl("applied"),
            "unit_of_measurement": "A",
            "device_class": "current",
            "state_class": "measurement",
        }),
        _entity("sensor", "session_energy", {
            "name": "Session energy",
            "state_topic": T_STATE,
            "value_template": tpl("session_wh"),
            "unit_of_measurement": "Wh",
            "device_class": "energy",
            "state_class": "total_increasing",
        }),
        _entity("sensor", "energy_total", {
            "name": "Total energy",
            "state_topic": T_STATE,
            "value_template": tpl("energy_total"),
            "unit_of_measurement": "kWh",
            "device_class": "energy",
            "state_class": "total_increasing",
            "suggested_display_precision": 2,
        }),
        _entity("sensor", "evse_state", {
            "name": "EVSE state",
            "state_topic": T_STATE,
            "value_template": tpl("evse_state"),
            "icon": "mdi:state-machine",
        }),
        _entity("binary_sensor", "link", {
            "name": "Wallbox link problem",
            "state_topic": T_STATE,
            "value_template": "{{ 'OFF' if value_json.evse_link else 'ON' }}",
            "device_class": "problem",
            "entity_category": "diagnostic",
        }),
        _entity("binary_sensor", "meter", {
            "name": "Energy meter problem",
            "state_topic": T_STATE,
            "value_template": "{{ 'OFF' if value_json.meter_ok else 'ON' }}",
            "device_class": "problem",
            "entity_category": "diagnostic",
        }),
        _entity("sensor", "meter_voltage", {
            "name": "Supply voltage",
            "state_topic": T_STATE,
            "value_template": tpl("vac"),
            "unit_of_measurement": "V",
            "device_class": "voltage",
            "state_class": "measurement",
            "entity_category": "diagnostic",
        }),
        _entity("sensor", "temperature", {
            "name": "Wallbox temperature",
            "state_topic": T_STATE,
            "value_template": tpl("temp_hi"),
            "unit_of_measurement": "\u00b0C",
            "device_class": "temperature",
            "state_class": "measurement",
            "entity_category": "diagnostic",
        }),
        _entity("sensor", "uptime", {
            "name": "Wallbox uptime",
            "state_topic": T_STATE,
            "value_template": tpl("uptime"),
            "unit_of_measurement": "s",
            "device_class": "duration",
            "entity_category": "diagnostic",
        }),
        _entity("sensor", "reboots", {
            "name": "Wallbox restarts",
            "state_topic": T_STATE,
            "value_template": tpl("reboots"),
            "state_class": "total_increasing",
            "icon": "mdi:restart-alert",
            "entity_category": "diagnostic",
        }),
        _entity("sensor", "link_down", {
            "name": "Wallbox unreachable for",
            "state_topic": T_STATE,
            "value_template": tpl("link_down_s"),
            "unit_of_measurement": "s",
            "state_class": "measurement",
            "icon": "mdi:lan-disconnect",
            "entity_category": "diagnostic",
        }),
        _entity("sensor", "limit_reason", {
            "name": "Limited by",
            "state_topic": T_STATE,
            "value_template": tpl("limit"),
            "icon": "mdi:speedometer-slow",
        }),
        # --- diagnostics, handy while tuning mode A -------------------------
        _entity("sensor", "inverter_power", {
            "name": "Multi RS AC output",
            "state_topic": T_STATE,
            "value_template": tpl("inv_out"),
            "unit_of_measurement": "W",
            "device_class": "power",
            "state_class": "measurement",
            "entity_category": "diagnostic",
        }),
        _entity("sensor", "battery_power", {
            "name": "Battery power",
            "state_topic": T_STATE,
            "value_template": tpl("batt_p"),
            "unit_of_measurement": "W",
            "device_class": "power",
            "state_class": "measurement",
            "entity_category": "diagnostic",
        }),
        _entity("sensor", "battery_soc", {
            "name": "Battery SOC",
            "state_topic": T_STATE,
            "value_template": tpl("soc"),
            "unit_of_measurement": "%",
            "device_class": "battery",
            "state_class": "measurement",
            "entity_category": "diagnostic",
        }),
        _entity("sensor", "pv_ac", {
            "name": "Fronius power",
            "state_topic": T_STATE,
            "value_template": tpl("pv_ac"),
            "unit_of_measurement": "W",
            "device_class": "power",
            "state_class": "measurement",
            "entity_category": "diagnostic",
        }),
        _entity("sensor", "pv_dc", {
            "name": "MPPT power",
            "state_topic": T_STATE,
            "value_template": tpl("pv_dc"),
            "unit_of_measurement": "W",
            "device_class": "power",
            "state_class": "measurement",
            "entity_category": "diagnostic",
        }),
        _entity("sensor", "car_gap", {
            "name": "Car shortfall",
            "state_topic": T_STATE,
            "value_template": tpl("car_gap"),
            "unit_of_measurement": "A",
            "device_class": "current",
            "state_class": "measurement",
            "entity_category": "diagnostic",
            "icon": "mdi:arrow-collapse-down",
        }),
        _entity("sensor", "drain_target", {
            "name": "Battery drain target",
            "state_topic": T_STATE,
            "value_template": tpl("drain_target"),
            "unit_of_measurement": "W",
            "state_class": "measurement",
            "entity_category": "diagnostic",
        }),
        _entity("sensor", "ac_frequency", {
            "name": "AC out frequency",
            "state_topic": T_STATE,
            "value_template": tpl("freq"),
            "unit_of_measurement": "Hz",
            "device_class": "frequency",
            "state_class": "measurement",
            "entity_category": "diagnostic",
        }),
    ]
    return out


class HaMqtt(object):
    def __init__(self, controller, log):
        self.ctl = controller
        self.log = log
        self.connected = False
        self.down_since = None
        import paho.mqtt.client as mqtt
        self.mqtt = mqtt
        client = None
        try:
            client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1,
                                 client_id="venus_evse_manager")
        except (AttributeError, TypeError):
            client = mqtt.Client(client_id="venus_evse_manager")
        self.client = client
        if MQTT_USER:
            self.client.username_pw_set(MQTT_USER, MQTT_PASS)
        self.client.will_set(T_AVAIL, "offline", retain=True)
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.on_disconnect = self._on_disconnect

    def watchdog(self):
        """
        Force a reconnect if the broker link has been down for a while.

        paho reconnects on its own most of the time, but when it does not, the
        symptom is nasty: the control loop keeps running and charging keeps
        working, while Home Assistant shows every entity as unavailable because
        the last will fired. From the outside that is indistinguishable from
        the script having died.
        """
        if self.connected:
            self.down_since = None
            return
        now = time.monotonic()
        if self.down_since is None:
            self.down_since = now
            return
        if now - self.down_since < 45.0:
            return
        self.down_since = now
        self.log.warning("MQTT down for 45s, forcing reconnect")
        try:
            self.client.reconnect()
        except Exception as exc:
            self.log.warning("MQTT reconnect failed: %s", exc)

    def start(self):
        try:
            self.client.connect_async(MQTT_HOST, MQTT_PORT, keepalive=30)
        except Exception as exc:
            self.log.error("mqtt connect_async: %s", exc)
        self.client.reconnect_delay_set(min_delay=1, max_delay=30)
        self.client.loop_start()

    def _on_connect(self, client, userdata, flags, rc, properties=None):
        if rc != 0:
            self.log.error("mqtt connect refused rc=%s", rc)
            return
        self.connected = True
        self.log.info("mqtt connected to %s", MQTT_HOST)
        for topic, payload in discovery_payloads():
            client.publish(topic, payload, retain=True)
        for component, object_id in RETIRED_ENTITIES:
            client.publish("%s/%s/victron_evse/%s/config"
                           % (HA_PREFIX, component, object_id), "", retain=True)
        client.publish(T_AVAIL, "online", retain=True)
        client.subscribe([(T_MODE + "/set", 0),
                          (T_MANUAL + "/set", 0),
                          (T_ENABLE + "/set", 0)])
        self.publish_controls()

    def _on_disconnect(self, client, userdata, rc, properties=None, reason=None):
        self.connected = False
        self.log.warning("mqtt disconnected rc=%s", rc)

    def _on_message(self, client, userdata, msg):
        try:
            payload = msg.payload.decode("utf-8", "ignore").strip()
        except Exception:
            return
        topic = msg.topic
        if topic == T_MODE + "/set":
            self.ctl.set_mode(LABEL_TO_MODE.get(payload, payload[:1].upper()))
        elif topic == T_MANUAL + "/set":
            try:
                self.ctl.set_manual(float(payload))
            except ValueError:
                pass
        elif topic == T_ENABLE + "/set":
            self.ctl.set_enabled(payload.upper() in ("ON", "TRUE", "1"))
        self.publish_controls()

    def publish_controls(self):
        if not self.connected:
            return
        self.client.publish(T_MODE + "/state",
                            MODE_LABELS.get(self.ctl.mode, MODE_LABELS["A"]),
                            retain=True)
        self.client.publish(T_MANUAL + "/state",
                            "%.0f" % self.ctl.manual_a, retain=True)
        self.client.publish(T_ENABLE + "/state",
                            "ON" if self.ctl.enabled else "OFF", retain=True)

    def publish_state(self, payload):
        if not self.connected:
            return
        self.client.publish(T_STATE, json.dumps(payload), retain=False)


# =============================================================================
#  Controller
# =============================================================================

class Controller(object):
    def __init__(self, ve, log):
        self.ve = ve
        self.log = log
        self.lock = threading.Lock()
        self.state_lock = threading.Lock()   # state.json: MQTT and control thread

        self.mode = "A"
        self.manual_a = 16.0
        self.enabled = True
        # the car's shortfall (offered minus measured amps), learned while charging
        self.gap_a = max(0.0, min(A_GAP_MAX_A, float(A_GAP_START_A)))
        self.load_state()
        self.gap_saved = self.gap_a
        self.gap_offer = None
        self.gap_since = None
        self.gap_amps = []
        self.gap_session = None       # per charge: start, base value, car limits itself
        self.gap_learnable = False    # set by step(): the offer does not follow the battery

        self.applied_a = 0.0
        self.paused = True
        self.soc_block = False
        self.last_up = -1e6
        self.last_cut = -1e6
        self.stop_started = None      # graceful wind-down began at
        self.stop_reason = ""         # ... and why
        self.below_min_since = None   # target has been under ABS_MIN_A since
        self.power_pause_at = None    # paused for lack of power at
        self.limit = "startup"

        # wallbox
        self.gw = Gateway(EVSE_IP, EVSE_HTTP_PORT, EVSE_TIMEOUT_S)
        self.wish = None              # (charge, amps on the wire) for the gateway
        self.sent = None              # the last wish the gateway accepted
        self.evse = {}
        self.evse_fail = 0
        self.evse_read_at = 0.0
        self.link_lost_at = None
        self.last_good = None         # last status seen before contact was lost
        self.reboots = 0
        self.prev_uptime = None
        self.fw_seen = None

        # meter
        self.meter = None
        if METER_DEVICE:
            self.meter = EnergyMeter(log, ModbusSerial(
                METER_DEVICE, METER_BAUD, METER_UNIT, METER_TIMEOUT_S))
        self.meter_w = None           # last measured power, shown on the gateway page
        self.meter_est_warned = False
        self.tare_w = None
        self.overdraw = 0
        self.delivery_checked = False
        self.meter_kwh = None         # the meter's own counter
        self.meter_kwh_at = None
        self.energy_read_at = -1e6
        self.energy_low = 0

        self.mqtt = None
        self.stop_flag = threading.Event()

    # ---- persisted user settings -------------------------------------------

    def load_state(self):
        try:
            with open(STATE_FILE, "r") as handle:
                data = json.load(handle)
            self.mode = data.get("mode", self.mode)
            if self.mode not in MODES:
                self.mode = "A"
            self.manual_a = max(ABS_MIN_A,
                                min(ABS_MAX_A,
                                    float(data.get("manual_a",
                                                   self.manual_a))))
            self.enabled = bool(data.get("enabled", self.enabled))
            if "gap_a" in data:
                self.gap_a = max(0.0, min(A_GAP_MAX_A, float(data["gap_a"])))
        except Exception:
            pass

    def save_state(self):
        with self.state_lock:
            self._save_state()

    def _save_state(self):
        try:
            tmp = STATE_FILE + ".tmp"
            with open(tmp, "w") as handle:
                json.dump({"mode": self.mode,
                           "manual_a": self.manual_a,
                           "enabled": self.enabled,
                           "gap_a": round(self.gap_a, 2)}, handle)
            os.replace(tmp, STATE_FILE)
        except Exception as exc:
            self.log.warning("state save failed: %s", exc)

    def set_mode(self, mode):
        if mode in MODES:
            with self.lock:
                self.mode = mode
                self.soc_block = False
            self.log.info("mode -> %s", MODE_LABELS[mode])
            self.save_state()

    def set_manual(self, amps):
        amps = max(ABS_MIN_A, min(ABS_MAX_A, round(amps)))
        with self.lock:
            self.manual_a = amps
        self.log.info("manual current -> %.0f A", amps)
        self.save_state()

    def set_enabled(self, flag):
        with self.lock:
            self.enabled = bool(flag)
        self.log.info("master enable -> %s", flag)
        self.save_state()

    # ---- wallbox I/O -----------------------------------------------------------

    @staticmethod
    def calibrate(amps):
        """
        Add the pilot offset so the car actually receives what we intended.

        Applied at the wire only - everything upstream (control loop, logs,
        Home Assistant) keeps working in real amps. Not applied to the stop
        taper: "offer 6 A to make the car let go" must put 6 A on the pilot,
        below the car's minimum, or the car never lets go.
        """
        if not PILOT_CAL_A or amps <= 0:
            return amps
        if amps <= STOP_TAPER_A + 0.01:
            return amps
        # never above the hardware ceiling - the gateway caps at its maximum
        return min(ABS_MAX_A, amps + PILOT_CAL_A)

    def push_evse(self, amps, enable):
        """
        Record what the wallbox should do now. A change is sent to the gateway
        straight away - a reduction must not wait for the next poll. An
        unchanged wish is repeated by refresh_evse() every EVSE_POLL_S, which
        is also the gateway's heartbeat.
        """
        wire = round(max(EVSE_HW_MIN_A, min(ABS_MAX_A, self.calibrate(amps))), 1)
        self.wish = (bool(enable), wire)
        if self.stop_flag.is_set():
            return                    # shutting down: shutdown() sends the final pause
        if self.wish != self.sent and self.evse_fail == 0:
            self.gw.command(self.wish[0], self.wish[1], self.meter_w,
                            self.energy_wh())
            self.sent = self.wish

    def refresh_evse(self):
        """
        One exchange with the gateway: our current wish goes out (heartbeat),
        the wallbox status comes back. Before the first decision there is no
        wish yet, so the very first exchange only reads.
        """
        if self.wish is None or self.stop_flag.is_set():
            data = self.gw.status()
        else:
            data = self.gw.command(self.wish[0], self.wish[1], self.meter_w,
                                   self.energy_wh())
            self.sent = self.wish
        if not data["link"]:
            raise LinkError("the gateway answers, but the EVSE board does not "
                            "answer the gateway (%s)" % data["reason"])
        data = self.check_overdraw(self.apply_tare(self.apply_meter(data)))
        self.meter_w = data["power"] if data.get("meter_ok") else None
        return data

    def note_wallbox_health(self, data):
        """Spot a gateway restart: its uptime going backwards is proof."""
        up = data.get("uptime")
        if up is None:
            return
        prev, self.prev_uptime = self.prev_uptime, up
        if prev is not None and up + 5 < prev:
            self.reboots += 1
            record_incident(
                "gateway RESTARTED (uptime %ds -> %ds); state=%s power=%.0fW "
                "setpoint=%.1fA" % (prev, up, data.get("state"),
                                    data.get("power") or 0.0,
                                    data.get("current_set") or 0.0))
            self.log.warning("wallbox gateway restarted (uptime reset to %ds)",
                             up)

    def check_delivery(self):
        """
        Once per session, compare what we asked for against what arrived.

        Three numbers separate the possible faults, and they are impossible to
        untangle from watts alone:
          measured current  ~= commanded  -> the pilot is fine
          measured voltage  <  nominal    -> the supply is sagging, not the car
          measured current  <  commanded  -> genuinely a pilot / vehicle issue
        """
        if self.delivery_checked or self.paused:
            return
        ev = self.evse
        if not ev.get("charging"):
            return
        amps = float(ev.get("curr") or 0.0)
        volts = float(ev.get("volt") or 0.0)
        watts = float(ev.get("power") or 0.0)
        if amps <= 0.5 or self.applied_a < ABS_MIN_A:
            return
        self.delivery_checked = True
        deficit = self.applied_a - amps
        self.log.info(
            "DELIVERY CHECK  asked %.1f A -> car took %.2f A (%.2f A short) "
            "at %.1f V = %.0f W (usual shortfall %.2f A)", self.applied_a,
            amps, deficit, volts, watts, self.gap_a)
        if volts > 1 and volts < 215:
            self.log.warning(
                "  supply is only %.0f V under load. Low power at the right "
                "current is a VOLTAGE problem, not the car.", volts)
        elif deficit > A_GAP_MAX_A + 0.5:
            self.log.warning(
                "  car is %.2f A short at a healthy %.0f V - more than usual. "
                "It may be limiting itself (nearly full, cold battery).",
                deficit, volts)

    # ---- energy meter ------------------------------------------------------

    def apply_meter(self, data):
        """
        Fill in the measured quantities (volt, curr, power) from the SDM120.
        State and setpoint come from the gateway, which is the only thing
        that knows them.
        """
        reading = self.meter.read() if self.meter is not None else None
        if reading is None:
            return self._no_meter(data)
        volts, amps, watts = reading
        if self.meter_est_warned:
            self.meter_est_warned = False
            self.log.info("energy meter back: %.1f V %.2f A %.0f W",
                          volts, amps, watts)
        data["meter_ok"] = True
        data["volt"] = volts if volts > 50 else NOMINAL_VAC
        data["curr"] = amps
        data["power"] = watts
        now = time.monotonic()
        if now - self.energy_read_at >= METER_ENERGY_EVERY_S:
            self.energy_read_at = now
            self.note_energy(self.meter.read_energy(), now)
        return data

    def note_energy(self, kwh, now):
        """Keep the meter's counter. A counter never goes back - unless the
        meter was replaced or reset, which three readings in a row confirm."""
        if kwh is None:
            return
        if self.meter_kwh is not None and kwh < self.meter_kwh - 0.01:
            self.energy_low += 1
            if self.energy_low < 3:
                return
            self.log.warning("energy meter counter went back from %.2f to "
                             "%.2f kWh - meter replaced or reset? Using the "
                             "new value.", self.meter_kwh, kwh)
        elif self.meter_kwh is None:
            self.log.info("energy meter counter: %.2f kWh", kwh)
        self.energy_low = 0
        self.meter_kwh = kwh
        self.meter_kwh_at = now

    def energy_wh(self):
        """The meter's counter in Wh for the gateway page, None if not fresh."""
        if self.meter_kwh is None or self.meter_kwh_at is None:
            return None
        if time.monotonic() - self.meter_kwh_at > 120.0:
            return None
        return self.meter_kwh * 1000.0

    def learn_gap(self):
        """
        How much less than the offer the car really takes, measured while it
        charges steadily and follows the pilot.

        Samples are taken only while the offer does NOT follow the battery:
        modes B, C and D, and mode A while its floor or cap sets the current.
        During solar tracking the offer depends on this estimate itself.

        A car that limits its own current (nearly full, cold battery) says
        nothing about the pilot, so learning stops for the rest of the charge
        as soon as the car does not follow a higher offer, or its current
        sinks while the offer stays the same. The current must also be
        steady (8 s within 0.15 A, one sample per such window), and one charge
        can move the value by at most 0.5 A.
        """
        ev = self.evse
        charging = (not self.paused and self.stop_started is None
                    and ev.get("charging") and ev.get("meter_ok")
                    and self.applied_a >= ABS_MIN_A)
        if not charging:
            self.gap_offer = None
            self.gap_session = None
            if not ev.get("charging") and \
                    abs(self.gap_a - self.gap_saved) >= 0.05:
                # end of a charge: keep what was learned
                self.gap_saved = self.gap_a
                self.log.info("car shortfall learned: %.2f A", self.gap_a)
                self.save_state()
            return
        now = time.monotonic()
        if self.gap_session is None:      # a charge has started
            self.gap_session = {"start": now, "base": self.gap_a,
                                "capped": False, "ref": None}
        sess = self.gap_session
        if not self.gap_learnable or sess["capped"]:
            self.gap_offer = None
            return
        if now - sess["start"] < GAP_START_S:
            return                        # the car is still starting up
        offered = float(ev.get("offered") or 0.0)
        amps = float(ev.get("curr") or 0.0)
        if offered < EVSE_HW_MIN_A or amps < 1.0:
            self.gap_offer = None
            return
        if self.gap_offer is None or abs(offered - self.gap_offer) > 0.25:
            self.gap_offer = offered      # new offer: give the car time to follow
            self.gap_since = now
            self.gap_amps = []
            return
        self.gap_amps = (self.gap_amps + [amps])[-8:]
        if now - self.gap_since < GAP_SETTLE_S or len(self.gap_amps) < 8:
            return
        if max(self.gap_amps) - min(self.gap_amps) > 0.15:
            return                        # the car is still changing its current
        # one sample per 8 steady seconds: the mean of that window
        amps = sum(self.gap_amps) / len(self.gap_amps)
        self.gap_amps = []
        # Does the car follow the pilot? Compared with the last steady point
        # where it did: a higher offer must raise its current by at least
        # half as much, and at the same offer its current must not sink.
        ref = sess["ref"]
        if ref is None:
            sess["ref"] = (offered, amps)
        else:
            d_off, d_amp = offered - ref[0], amps - ref[1]
            if (d_off > 0.15 and d_amp < 0.5 * d_off) or \
                    (abs(d_off) <= 0.15 and d_amp < -0.3):
                sess["capped"] = True
                self.log.info("car limits its own current (offer %.2f -> "
                              "%.2f A, it took %.2f -> %.2f A): shortfall "
                              "learning paused until the next charge",
                              ref[0], offered, ref[1], amps)
                return
            if abs(d_off) > 0.15:
                sess["ref"] = (offered, amps)
        gap = offered - amps
        if gap < -0.5 or gap > A_GAP_MAX_A + 0.5:
            return
        gap = max(0.0, min(A_GAP_MAX_A, gap))
        self.gap_a += (gap - self.gap_a) * 0.05
        lo, hi = sess["base"] - 0.5, sess["base"] + 0.5
        self.gap_a = max(0.0, lo, min(A_GAP_MAX_A, hi, self.gap_a))

    def _no_meter(self, data):
        """
        No meter reading: estimate from the current we are asking for. Mode A
        regulates on this number, so an honest estimate beats nothing.

        Release detection during a stop does not depend on it: the car going
        from state C to B is a digital signal, not a measurement.
        """
        charging = data.get("charging") and not self.paused
        amps = max(0.0, self.applied_a - self.gap_a) if charging else 0.0
        if self.meter is not None and not self.meter_est_warned:
            self.meter_est_warned = True
            self.log.warning("energy meter unavailable - estimating charge "
                             "power from the setpoint until it returns")
        data["meter_ok"] = False
        data["volt"] = NOMINAL_VAC
        data["curr"] = amps
        data["power"] = amps * NOMINAL_VAC
        return data

    def check_overdraw(self, data):
        """
        The meter is trusted now, so a reading above what the pilot permits is
        no longer "a bad sample to be clamped" - it is a real fault worth
        shouting about. A car cannot legally draw more than the pilot offers,
        so if this fires, something is genuinely wrong: a welded contactor, a
        vehicle ignoring the pilot, or the meter measuring the wrong circuit.

        Deliberately does NOT modify the reading. Hiding it was only ever a
        defence against a lying ADC.
        """
        if self.paused or self.applied_a <= 0.0 or not data.get("meter_ok"):
            return data
        volts = float(data.get("volt") or 0.0)
        if volts < 180.0 or volts > 270.0:
            volts = NOMINAL_VAC
        ceiling = (self.applied_a + 2.0) * volts
        watts = float(data.get("power") or 0.0)
        if watts > ceiling:
            self.overdraw += 1
            if self.overdraw in (1, 20, 200):
                self.log.error(
                    "OVERDRAW: meter reads %.0f W while the pilot offers only "
                    "%.1f A at %.0f V (max %.0f W). The car cannot do this - "
                    "check the contactor and that the meter is on the right "
                    "circuit.", watts, self.applied_a, volts, ceiling)
                record_incident(
                    "overdraw: %.0f W measured, %.1f A offered" % (watts, self.applied_a))
        else:
            self.overdraw = 0
        return data

    def apply_tare(self, data):
        """
        Subtract the wallbox's own consumption from an upstream meter.

        Learned rather than configured: whenever the car is not drawing, what
        the meter still reads IS the wallbox's standby draw, so we take that as
        the offset. Slowly, with an exponential average, so one odd sample
        cannot bias it - and never above METER_TARE_MAX_W, since a big reading
        while "not charging" means something else is going on and must not be
        mistaken for a baseline.
        """
        if not data.get("meter_ok"):
            return data
        mode = str(METER_TARE).lower()
        if mode == "off":
            return data
        watts = float(data.get("power") or 0.0)

        if mode == "auto":
            if not data.get("charging") and 0.0 <= watts <= METER_TARE_MAX_W:
                if self.tare_w is None:
                    self.tare_w = watts
                    self.log.info("meter standby draw measured at %.1f W "
                                  "(subtracted from charging power)", watts)
                else:
                    self.tare_w += (watts - self.tare_w) * 0.05
        else:
            try:
                self.tare_w = float(METER_TARE)
            except (TypeError, ValueError):
                return data

        if not self.tare_w:
            return data
        data["power_gross"] = watts
        data["power"] = max(0.0, watts - self.tare_w)
        volts = float(data.get("volt") or 0.0)
        if volts > 50:
            data["curr"] = max(0.0, float(data.get("curr") or 0.0)
                               - self.tare_w / volts)
        return data

    # ---- measurement snapshot ----------------------------------------------

    def measure(self):
        ve = self.ve
        vac = ve.get_any(("multi", "vebus", "inverter"), "/Ac/Out/L1/V")
        if not vac or vac < 180 or vac > 270:
            vac = NOMINAL_VAC
        freq = ve.get_any(("multi", "vebus", "inverter"), "/Ac/Out/L1/F")
        if not freq or freq < 45 or freq > 65:
            freq = 50.0

        soc = ve.get("system", "/Dc/Battery/Soc")
        batt_p = ve.get("system", "/Dc/Battery/Power")
        pv_ac = ve.get("system", "/Ac/PvOnOutput/L1/Power") or 0.0
        pv_dc = ve.get("system", "/Dc/Pv/Power") or 0.0

        load = ve.get("system", "/Ac/ConsumptionOnOutput/L1/Power")
        if load is None:
            load = ve.get("system", "/Ac/Consumption/L1/Power")

        inv_out = ve.get_any(("multi", "vebus", "inverter"), "/Ac/Out/L1/P")
        if inv_out is None and load is not None:
            # fall back: whatever the loads take that the Fronius does not cover
            inv_out = load - pv_ac
        if load is None and inv_out is not None:
            load = inv_out + pv_ac

        return {
            "vac": float(vac),
            "freq": float(freq),
            "soc": soc,
            "batt_p": batt_p,
            "pv_ac": max(0.0, float(pv_ac)),
            "pv_dc": max(0.0, float(pv_dc)),
            "load": load,
            "inv_out": inv_out,
        }

    # ---- mode A -------------------------------------------------------------

    def drain_target(self, soc, freq):
        """
        How much the battery is allowed to give the car, beyond the sun.

        Normally: nothing. Mode A means "charge on sun, with an 8 A floor" -
        the battery is not a second source to be drawn down while the sun is
        perfectly capable. The floor is enforced separately, so a dull day
        still gets 8 A; the battery covers only that shortfall.

        The exception is a nearly full pack. Victron raises AC output frequency
        to throttle the Fronius once the battery has nowhere to put the energy,
        and the Fronius then stops producing. Taking a deliberate trickle out
        of the battery keeps it off the ceiling and keeps the Fronius awake -
        which yields far more energy than the trickle costs.
        """
        if soc is None:
            return 0.0
        # The Multi throttles the Fronius and the MPPTs as soon as the
        # battery stops taking full charge - seen at 97 % SOC with 52.7 Hz,
        # below A_SOC_DRAIN_START. Frequency shift is the real sign that sun
        # is going to waste, so it always allows the drain, whatever the SOC.
        throttled = freq >= FREQ_SHIFT_HZ
        if soc < A_SOC_DRAIN_START and not throttled:
            return 0.0
        target = A_DRAIN_BASE_W
        if soc >= A_DRAIN_SOC_MAX:
            target = A_DRAIN_W_MAX
        elif soc >= A_DRAIN_SOC_HI:
            target = A_DRAIN_W_HI
        if freq >= FREQ_SHIFT_HZ:
            # Confirmed throttling: there is free energy going to waste.
            target += A_DRAIN_FREQ_BONUS_W
        return target

    def mode_a_target(self, m, ev_p, drain):
        """
        Closed loop on battery power.

        batt_p is positive while charging, negative while discharging. We want
        it to sit at -drain. The error tells us directly how many watts the car
        may take on top of what it takes now:

            delta = batt_p + drain

        Positive delta -> the battery is charging (or discharging less than the
        target), so there is room to give the car more. Negative delta -> we are
        pulling too hard, back off. No solar forecast, no surplus arithmetic,
        no dependence on the Fronius reporting correctly.
        """
        if m["batt_p"] is None:
            return None
        delta = (m["batt_p"] + drain) * A_GAIN
        return ev_p + delta

    # ---- main control step --------------------------------------------------

    def step(self, now):
        with self.lock:
            mode = self.mode
            manual_a = self.manual_a
            enabled = self.enabled
        cfg = MODES[mode]

        m = self.measure()
        vac = m["vac"]
        ev = self.evse
        ev_p = float(ev.get("power") or 0.0)
        connected = bool(ev.get("connected"))
        stale = self.ve.stale() or m["batt_p"] is None or m["inv_out"] is None

        # ---- hard gates -----------------------------------------------------
        # Charging forbidden -> the gateway is told to pause.
        if not enabled or mode == "E":
            return self.apply(now, 0.0, "disabled", m, allow=False)
        # Nothing plugged in -> permitted, just nothing to do.
        if not connected:
            self.soc_block = False
            return self.apply(now, 0.0, "no vehicle", m, allow=True)
        if ev.get("errors"):
            return self.apply(now, 0.0,
                              "evse error: %s" % format_errors(ev["errors"]),
                              m, allow=False, immediate=True)
        if not ev.get("switch", True):
            # The on/off switch on the gateway's own web page is off. The
            # gateway has stopped charging already; follow it, so charging
            # restarts gently from RESUME_A when the switch goes back on.
            return self.apply(now, 0.0, "switched off on the wallbox page", m,
                              allow=False, immediate=True)
        if ev.get("holdoff"):
            # The gateway waits before it lets the car restart (after a fault,
            # or its EEPROM write limit). Pause too, and restart gently later.
            return self.apply(now, 0.0, "wallbox waits %ds: %s"
                              % (ev["holdoff"], ev.get("reason")), m,
                              allow=False, immediate=True)

        soc = m["soc"]
        if soc is not None:
            if soc < cfg["soc_min"]:
                self.soc_block = True
            elif soc >= cfg["soc_min"] + SOC_HYSTERESIS:
                self.soc_block = False
        if self.soc_block:
            return self.apply(now, 0.0, "SOC floor %.0f%%" % cfg["soc_min"], m,
                              allow=False)

        if stale:
            # We cannot see the system any more. Do not guess - hold the floor.
            return self.apply(now, min(ABS_MIN_A, cfg["i_max"]), "no D-Bus data", m)

        # ---- desired power --------------------------------------------------
        limit = "mode cap"
        drain = 0.0
        if mode == "D":
            desired_p = manual_a * vac
        elif mode == "A":
            drain = self.drain_target(soc, m["freq"])
            desired_p = self.mode_a_target(m, ev_p, drain)
            if desired_p is None:
                desired_p = cfg["i_floor"] * vac
            limit = "solar tracking"
        else:
            desired_p = cfg["i_max"] * vac

        # How much of the car's power may come out of the battery. Solar surplus
        # is everything the arrays make minus everything else in the house.
        other_loads = max(0.0, (m["load"] or 0.0) - ev_p)
        pv_total = m["pv_ac"] + m["pv_dc"] * DC_TO_AC_EFF
        solar_share = max(0.0, pv_total - other_loads)
        # In mode A the battery's whole allowance IS the drain target, so the
        # car is held to the solar surplus plus that trickle. The 8 A floor
        # below is what lets the battery cover a dull day, and nothing else.
        batt_allow = drain if mode == "A" else cfg["batt_ev_w"]
        batt_share_cap = solar_share + batt_allow
        if batt_share_cap < desired_p:
            desired_p = batt_share_cap
            limit = "battery share cap"

        # Floor: mode A keeps 8 A running off the battery at night.
        floor_p = cfg["i_floor"] * vac
        if desired_p < floor_p:
            desired_p = floor_p
            limit = "mode floor"

        # ---- hard protection headroom ---------------------------------------
        inv_room = (INV_MAX_W - INV_RESERVE_W) - m["inv_out"]
        batt_room = BATT_HARD_MAX_W + m["batt_p"]
        room = min(inv_room, batt_room)
        headroom_p = ev_p + room
        if headroom_p < desired_p:
            desired_p = headroom_p
            limit = "inverter headroom" if inv_room <= batt_room else "battery headroom"

        target_a = desired_p / vac
        # Mode A works in what the car REALLY takes, so it asks for the car's
        # shortfall on top - but never for more than the measured headroom:
        # protection always works in plain measured watts, so an estimate
        # that is too high can never push the Multi or the battery over.
        if mode == "A" and not PILOT_CAL_A and self.gap_a > 0:
            target_a += self.gap_a
            if target_a > headroom_p / vac:
                target_a = headroom_p / vac
                limit = ("inverter headroom" if inv_room <= batt_room
                         else "battery headroom")
        if target_a > cfg["i_max"]:
            target_a = cfg["i_max"]
            limit = "mode cap"
        target_a = min(target_a, ABS_MAX_A)
        # the shortfall is learned only while the offer does not follow the battery
        self.gap_learnable = mode != "A" or limit in ("mode floor", "mode cap")

        # ---- slew shaping ---------------------------------------------------
        # While paused, apply() decides the restart current - do not rate limit
        # here or the target can never climb past the restart threshold.
        if self.paused:
            pass
        elif target_a < self.applied_a - 0.05:
            # Reductions are immediate and take a little extra for safety.
            if room < 0:
                target_a = max(0.0, target_a - CUT_MARGIN_A)
                self.last_cut = now
                limit = "protective cut"
        elif target_a > self.applied_a + 0.05:
            if now - self.last_cut < RECOVERY_HOLD_S:
                target_a = self.applied_a
                limit = "recovery hold"
            elif now - self.last_up < UP_SETTLE_S:
                target_a = self.applied_a
            else:
                near_limit = m["inv_out"] > (INV_MAX_W - INV_RESERVE_W) * 0.85
                step = UP_STEP_PROBE_A if near_limit else UP_STEP_A
                target_a = min(target_a, self.applied_a + step)
                self.last_up = now
                if near_limit:
                    limit = "probing Fronius"

        urgent = room < 0 or limit == "protective cut"
        return self.apply(now, target_a, limit, m,
                          regulated=(mode != "D"), urgent=urgent)

    def apply(self, now, target_a, limit, m, allow=None,
              regulated=False, urgent=False, immediate=False):
        """
        allow controls the "charging enabled" flag independently of the current
        setpoint - "no car plugged in" is not a reason to forbid charging.

        regulated marks the solar-tracking path, where the target can drift
        under ABS_MIN_A simply because a cloud went over. There we ride it out
        at the minimum for MIN_DWELL_S before stopping, and once stopped we
        stay stopped for REPAUSE_HOLD_S, so the car is not started and stopped
        every few seconds. urgent bypasses both: an overload must act now.
        """
        self.limit = limit
        if allow is None:
            allow = target_a >= ABS_MIN_A

        if not regulated:
            # Hard gate (disabled, no vehicle, error, SOC floor). Immediate,
            # and it clears the power-related timers.
            self.below_min_since = None
            self.power_pause_at = None
            if target_a < ABS_MIN_A:
                return self._pause(now, allow, limit, m, immediate=immediate)
        else:
            if target_a < ABS_MIN_A:
                if urgent:
                    # A protective cut is not solar chatter. Use the short
                    # recovery hold so the car restarts promptly once the
                    # water heater (or whatever it was) goes away, rather than
                    # sitting idle for REPAUSE_HOLD_S.
                    self.below_min_since = None
                    self.power_pause_at = None
                    self.last_cut = now
                    return self._pause(now, allow, limit, m)
                if self.paused or self.stop_started is not None:
                    # paused, or a stop is under way: see it through
                    return self._pause(now, allow, self.stop_reason
                                       if self.stop_started is not None
                                       else limit, m)
                if self.below_min_since is None:
                    self.below_min_since = now
                held = now - self.below_min_since
                if held < MIN_DWELL_S:
                    # Keep the session alive at the floor and see if the sun
                    # comes back. Cheaper than a stop/start cycle.
                    target_a = ABS_MIN_A
                    self.limit = "%s (holding %.0f A, %.0fs)" % (
                        limit, ABS_MIN_A, MIN_DWELL_S - held)
                else:
                    self.below_min_since = None
                    self.power_pause_at = now
                    return self._pause(now, allow, "not enough power", m)
            else:
                self.below_min_since = None
                if self.paused and self.power_pause_at is not None:
                    waited = now - self.power_pause_at
                    if waited < REPAUSE_HOLD_S:
                        self.limit = "restart in %.0fs" % (
                            REPAUSE_HOLD_S - waited)
                        return self._pause(now, allow, self.limit, m)
                    self.power_pause_at = None

        if self.paused:
            self.applied_a = min(target_a,
                                 max(RESUME_A, MODES[self.mode]["i_floor"]))
            self.paused = False
            self.stop_started = None
            self.last_up = now
            self.log.info("resume charging at %.1f A (%s)",
                          self.applied_a, self.limit)
        else:
            if self.stop_started is not None:
                # A stop was under way, but charging goes on after all (for
                # example switched back on within seconds). Forget its start
                # time, or the next stop would skip the taper.
                self.stop_started = None
                self.log.info("stop called off, charging continues (%s)",
                              self.limit)
            if target_a < self.applied_a - DOWN_DEADBAND_A or \
                    target_a > self.applied_a + WRITE_DEADBAND_A:
                self.applied_a = target_a
        self.applied_a = max(ABS_MIN_A, min(ABS_MAX_A, self.applied_a))
        self.push_evse(self.applied_a, True)
        return m

    def _pause(self, now, allow, limit, m, immediate=False):
        """
        Three-stage stop, driven by the measured current rather than a timer.

          1. Taper to STOP_TAPER_A, below the car's own minimum. The car gives
             up the session and opens ITS contactor - which is the only one
             that matters, because it is upstream of ours.
          2. Watch the meter until the power actually falls away.
          3. Only then remove the pilot and let the EVSE open the AC relay,
             now with nothing flowing through it.

        immediate=True skips all of this: a pilot fault, diode short or RCD
        trip means the pilot circuit is suspect and protecting relay contacts
        is not worth another second of charging.
        """
        power = float(self.evse.get("power") or 0.0)

        if not immediate and self.evse.get("charging") and not self.paused:
            if self.stop_started is None:
                self.stop_started = now
                self.stop_reason = limit
                self.log.info("stopping: tapering to %.1f A to release the car "
                              "(drawing %.0f W)", STOP_TAPER_A, power)
            elapsed = now - self.stop_started
            settled = power <= STOP_POWER_W and elapsed >= STOP_RAMP_S
            if not settled and elapsed < STOP_MAX_S:
                self.applied_a = STOP_TAPER_A
                self.limit = "%s (releasing car, %.0f W)" % (limit, power)
                self.push_evse(STOP_TAPER_A, True)
                return m
            if power > STOP_POWER_W:
                # The car ignored the taper for the full timeout. Record it:
                # this is the case where the contactor really does open under
                # load, and it is worth knowing how often it happens.
                self.log.warning(
                    "car still drawing %.0f W after %.1fs of taper - "
                    "opening contactor under load", power, elapsed)
                record_incident(
                    "contactor opened under load: %.0f W after %.1fs taper "
                    "(car did not release)" % (power, elapsed))
            else:
                self.log.info("car released after %.1fs (%.0f W), "
                              "opening contactor cleanly", elapsed, power)

        self.stop_started = None
        if not self.paused:
            self.log.info("pause charging (%s)", limit)
        self.paused = True
        self.applied_a = 0.0
        self.push_evse(STOP_TAPER_A, allow)
        return m

    # ---- publish ------------------------------------------------------------

    def publish(self, m):
        ev = self.evse
        payload = {
            "mode": self.mode,
            "mode_label": MODE_LABELS[self.mode],
            "enabled": self.enabled,
            "evse_state": ev.get("state", "?"),
            "connected": bool(ev.get("connected")),
            "charging": bool(ev.get("charging")),
            "power": round(float(ev.get("power") or 0.0)),
            "meter_tare_w": round(self.tare_w, 1) if self.tare_w else 0,
            "overdraw_count": self.overdraw,
            "current": round(float(ev.get("curr") or 0.0), 2),
            "applied": round(0.0 if self.paused else self.applied_a, 1),
            "setpoint": ev.get("current_set"),
            "offered": ev.get("offered"),
            "session_wh": ev.get("session_wh", 0),
            "session_s": ev.get("session_s", 0),
            "energy_total": (None if self.meter_kwh is None
                             else round(self.meter_kwh, 3)),
            "car_gap": round(self.gap_a, 2),
            "limit": self.limit,
            "wallbox_reason": ev.get("reason"),
            "soc": None if m.get("soc") is None else round(m["soc"], 1),
            "batt_p": None if m.get("batt_p") is None else round(m["batt_p"]),
            "inv_out": None if m.get("inv_out") is None else round(m["inv_out"]),
            "pv_ac": round(m.get("pv_ac", 0.0)),
            "pv_dc": round(m.get("pv_dc", 0.0)),
            "load": None if m.get("load") is None else round(m["load"]),
            "freq": round(m.get("freq", 50.0), 2),
            "vac": round(m.get("vac", NOMINAL_VAC), 1),
            "drain_target": round(self.drain_target(m.get("soc"), m.get("freq", 50.0)))
                            if self.mode == "A" else 0,
            "meter_ok": ev.get("meter_ok"),
            "evse_link": self.evse_fail == 0 and bool(ev),
            "evse_fails": self.evse_fail,
            "reboots": self.reboots,
            "uptime": ev.get("uptime"),
            "temp_hi": ev.get("temp_hi"),
            "temp_lo": ev.get("temp_lo"),
            "fw": ev.get("fw"),
            "link_down_s": (round(time.time() - self.link_lost_at)
                            if self.link_lost_at else 0),
        }
        if self.mqtt:
            self.mqtt.publish_state(payload)

    # ---- worker thread ------------------------------------------------------

    def run(self):
        last_publish = 0.0
        # Start the clock now, so the first status line appears once the
        # wallbox and MQTT have had a chance to come up.
        last_log = time.monotonic()
        announced = False
        while not self.stop_flag.is_set():
            started = time.monotonic()
            m = {}
            try:
                # The gateway is polled at EVSE_POLL_S while the control loop
                # runs at LOOP_S. Protection reacts on D-Bus data, which is the
                # fast signal, and a changed command is sent immediately.
                if started - self.evse_read_at >= EVSE_POLL_S:
                    self.evse_read_at = started
                    self.evse = self.refresh_evse()
                    self._read_ok(announced)
                    announced = True
                if self.evse and self.evse_fail == 0:
                    m = self.step(started) or {}
                else:
                    m = self.measure()
            except LinkError as exc:
                self._link_failed(exc)
                m = self.measure()
            except Exception as exc:
                self.log.exception("control loop error: %s", exc)
                m = {}

            now = time.monotonic()
            if now - last_publish >= 1.0:
                last_publish = now
                try:
                    self.publish(m or self.measure())
                except Exception as exc:
                    self.log.warning("publish failed: %s", exc)
            if now - last_log >= 60.0:
                last_log = now
                try:
                    self._status_line(m)
                except Exception:
                    pass

            sleep = LOOP_S - (time.monotonic() - started)
            if sleep > 0:
                self.stop_flag.wait(sleep)
        self.log.warning("control loop exiting")

    def _read_ok(self, announced):
        ev = self.evse
        if not announced:
            self.log.info("wallbox reachable: state=%s, gateway has %s at "
                          "%.1f A (%s)", ev.get("state"),
                          "charge" if ev.get("enabled") else "pause",
                          ev.get("current_set") or 0.0, ev.get("reason"))
        fw = ev.get("fw")
        if fw and fw != self.fw_seen:
            self.fw_seen = fw
            self.log.info("wallbox gateway firmware %s", fw)
        self.note_wallbox_health(ev)
        if not ev.get("charging"):
            self.delivery_checked = False
        else:
            self.check_delivery()
        self.learn_gap()
        self.last_good = dict(ev)
        if self.evse_fail:
            down = (time.time() - self.link_lost_at
                    if self.link_lost_at else 0.0)
            record_incident("link RESTORED after %.0fs (%d failed attempts)"
                            % (down, self.evse_fail))
            self.log.info("wallbox link restored after %.0fs (%d failures)",
                          down, self.evse_fail)
            self.link_lost_at = None
            if down >= RELINK_GENTLE_S and not self.paused:
                if ev.get("charging"):
                    # Still charging on the old current: bring it down to the
                    # restart current and let the loop climb again from there.
                    self.applied_a = min(self.applied_a, RESUME_A)
                    self.log.info("continuing gently after the outage (%.1f A)",
                                  self.applied_a)
                else:
                    # The gateway's watchdog stopped the car meanwhile, or it
                    # was unplugged: start again from the bottom.
                    self.paused = True
                    self.applied_a = 0.0
                    self.stop_started = None
                    self.log.info("restarting gently after the outage")
        self.evse_fail = 0

    def _link_failed(self, exc):
        if self.evse_fail == 0:
            # First failure of this outage: capture what the wallbox was doing
            # at the moment it stopped answering.
            self.link_lost_at = time.time()
            last = self.last_good or {}
            record_incident(
                "link LOST: %s | last seen state=%s power=%.0fW setpoint=%.1fA "
                "applied=%.1fA mode=%s"
                % (exc, last.get("state"), last.get("power") or 0.0,
                   last.get("current_set") or 0.0, self.applied_a, self.mode))
        self.evse_fail += 1
        if self.evse_fail == 1 or self.evse_fail % 60 == 0:
            self.log.warning("wallbox link: %s (failure %d)", exc,
                             self.evse_fail)
        self.sent = None
        self.limit = "wallbox unreachable"

    def _status_line(self, m):
        wish = self.wish
        self.log.info(
            "mode=%s applied=%.1fA wire=%s mqtt=%s evse=%s power=%.0fW "
            "carV=%.1f carI=%.2f inv=%s invV=%.1f batt=%s soc=%s "
            "pvac=%s pvdc=%s f=%.2f gap=%.2fA meter=%s limit=%s",
            self.mode, self.applied_a,
            ("%s %.1fA" % ("charge" if wish[0] else "pause", wish[1]))
            if wish else "-",
            "up" if (self.mqtt and self.mqtt.connected) else "DOWN",
            self.evse.get("state"),
            float(self.evse.get("power") or 0.0),
            float(self.evse.get("volt") or 0.0),
            float(self.evse.get("curr") or 0.0),
            m.get("inv_out"), m.get("vac", 0.0), m.get("batt_p"),
            m.get("soc"), m.get("pv_ac"), m.get("pv_dc"),
            m.get("freq", 0.0), self.gap_a,
            "-" if self.meter_kwh is None else "%.2fkWh" % self.meter_kwh,
            self.limit)

    def shutdown(self):
        """
        Called once the control thread has stopped. Pause the wallbox now
        rather than leaving it to the gateway's watchdog: a stopped service
        should not leave the car charging for three more minutes.
        """
        if self.meter is not None:
            self.meter.close()
        if not self.evse or self.evse_fail:
            return
        try:
            self.gw.command(False, max(EVSE_HW_MIN_A, ABS_MIN_A), None)
            self.log.info("wallbox paused on shutdown")
        except Exception as exc:
            self.log.warning("could not pause the wallbox on shutdown: %s", exc)


# =============================================================================
#  main
# =============================================================================

def process_start_time(pid):
    """Wall clock time a pid started, so we can spot a process running stale code."""
    try:
        with open("/proc/stat", "r") as handle:
            btime = 0.0
            for line in handle:
                if line.startswith("btime "):
                    btime = float(line.split()[1])
                    break
        if not btime:
            return None
        with open("/proc/%s/stat" % pid, "r") as handle:
            fields = handle.read().rsplit(") ", 1)[1].split()
        ticks = float(fields[19])          # field 22 overall, 0-based after comm
        hz = float(os.sysconf("SC_CLK_TCK") or 100)
        return btime + ticks / hz
    except Exception:
        return None


def parent_of(pid):
    """Return (ppid, parent cmdline) for a pid, or (None, '')."""
    try:
        with open("/proc/%s/status" % pid, "r") as handle:
            ppid = None
            for line in handle:
                if line.startswith("PPid:"):
                    ppid = line.split()[1]
                    break
        if not ppid:
            return (None, "")
        try:
            with open("/proc/%s/cmdline" % ppid, "rb") as handle:
                pcmd = handle.read().replace(b"\x00", b" ").decode(
                    "utf-8", "ignore").strip()
        except IOError:
            pcmd = ""
        return (ppid, pcmd)
    except Exception:
        return (None, "")


def doctor(log, applied, retired, cfg_err):
    """Duplicate instances, the gateway and the meter - one look at each."""
    print("=" * 62)
    print("evse_manager doctor   (script version %s)" % VERSION)
    try:
        mtime = os.path.getmtime(os.path.abspath(__file__))
        print("file: %s" % os.path.abspath(__file__))
        print("last written: %s"
              % time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(mtime)))
    except Exception:
        mtime = 0
    print("=" * 62)

    # 0. effective configuration
    if os.path.exists(CONFIG_FILE):
        print("config: %s (%s)"
              % (CONFIG_FILE,
                 ", ".join(sorted(applied)) if applied else "nothing applied"))
    else:
        print("config: %s NOT FOUND - using built-in defaults" % CONFIG_FILE)
    if retired:
        print("        no longer used, ignored: %s" % ", ".join(sorted(retired)))
    if cfg_err:
        print("        %s" % cfg_err)
    print("        wallbox gateway %s:%d   broker %s:%d   mqtt user %s"
          % (EVSE_IP, EVSE_HTTP_PORT, MQTT_HOST, MQTT_PORT,
             repr(MQTT_USER) if MQTT_USER else "(anonymous)"))
    print("=" * 62)

    # 1. duplicate instances
    me = os.getpid()
    mine = set()
    pid = str(me)
    for _ in range(12):
        ppid = parent_of(pid)[0]
        if not ppid or ppid in ("0", "1"):
            break
        mine.add(ppid)
        pid = ppid
    others = []
    try:
        for pid in os.listdir("/proc"):
            if not pid.isdigit() or int(pid) == me or pid in mine:
                continue
            try:
                with open("/proc/%s/cmdline" % pid, "rb") as handle:
                    cmd = handle.read().replace(b"\x00", b" ").decode(
                        "utf-8", "ignore")
            except IOError:
                continue
            if "evse_manager.py" in cmd:
                others.append((pid, cmd.strip()))
    except Exception as exc:
        print("could not scan /proc: %s" % exc)
    def supervised_by_service(pid):
        for _ in range(2):          # the run script may sit in between
            pid, pcmd = parent_of(pid)
            if not pid:
                return False
            if "supervise" in pcmd:
                return True
        return False
    supervised = [o for o in others if supervised_by_service(o[0])]
    if len(others) == 1 and supervised:
        # One copy, started by the service supervisor - as it should be.
        pid, cmd = others[0]
        started = process_start_time(pid)
        print("OK  the service is running (pid %s)" % pid)
        if started and mtime and started < mtime:
            print("!!  but it started BEFORE evse_manager.py was last written,")
            print("    so it still runs the old code. Restart it:")
            print("      svc -t /service/evse_manager")
    elif len(others) == 1:
        pid, cmd = others[0]
        print("!!  a copy of this script runs OUTSIDE the service (pid %s):" % pid)
        print("      %s" % cmd[:60])
        print("    If you started it by hand, stop it:  kill %s" % pid)
        print("    then start the service:  svc -u /service/evse_manager")
    elif others:
        print("!! %d copies of this script are running - one is too many:"
              % len(others))
        stale = False
        for pid, cmd in others:
            started = process_start_time(pid)
            note = ""
            if started and mtime and started < mtime:
                note = "  <-- STARTED BEFORE THE FILE WAS LAST WRITTEN"
                stale = True
            print("     pid %s  %s%s" % (pid, cmd[:52], note))
            # The parent is what actually matters: killing a child that has a
            # supervisor just gets you a new child a second later.
            ppid, pcmd = parent_of(pid)
            if ppid:
                print("       parent pid %s: %s" % (ppid, pcmd[:56] or "?"))
                if "supervise" in pcmd:
                    print("       ^ daemontools supervisor - kill THIS, or it")
                    print("         will keep respawning the script:")
                    print("           kill -9 %s" % ppid)
                elif ppid == "1":
                    print("       ^ parented to init: its supervisor already")
                    print("         exited, so a plain kill will stick.")
        if stale:
            print("")
            print("   That process is running OLD CODE held in memory. Editing")
            print("   the file does not change a process that is already up.")
        print("")
        print("   Stop it properly - killing the process is not enough,")
        print("   daemontools restarts it about a second later:")
        print("     svc -dx /service/evse_manager")
        print("     svc -dx /service/evse_manager/log")
        print("     rm -f /service/evse_manager")
        print("     sleep 3")
        print("     pkill -f evse_manager.py")
        print("     pgrep -af evse_manager.py    # must print nothing")
    else:
        print("--  the service is not running (start it: svc -u /service/evse_manager)")

    # 1b. orphaned daemontools supervisors
    orphans = []
    try:
        for pid in os.listdir("/proc"):
            if not pid.isdigit():
                continue
            try:
                with open("/proc/%s/cmdline" % pid, "rb") as handle:
                    cmd = handle.read().replace(b"\x00", b" ").decode(
                        "utf-8", "ignore").strip()
            except IOError:
                continue
            if not cmd.startswith("supervise"):
                continue
            try:
                cwd = os.readlink("/proc/%s/cwd" % pid)
            except OSError:
                continue
            if cwd.startswith("/data/evse_manager/service") or \
                    cwd.startswith("/service/evse_manager"):
                orphans.append((pid, cmd, cwd))
    except Exception:
        pass
    if orphans:
        print("")
        print("!! daemontools supervisor(s) for this service are running:")
        for pid, cmd, cwd in orphans:
            print("     pid %s  %s   cwd=%s" % (pid, cmd[:26], cwd))
        if not os.path.exists("/service/evse_manager"):
            print("   The /service symlink is already gone, so 'svc' cannot")
            print("   reach these. They must be killed directly:")
            print("     kill -9 %s" % " ".join(p for p, _, _ in orphans))
            print("   Kill the supervisor BEFORE the script, or it just")
            print("   respawns a fresh copy a second later.")

    # 2. any other python touching the wallbox
    strays = []
    try:
        for pid in os.listdir("/proc"):
            if not pid.isdigit() or int(pid) == me:
                continue
            try:
                with open("/proc/%s/cmdline" % pid, "rb") as handle:
                    cmd = handle.read().replace(b"\x00", b" ").decode(
                        "utf-8", "ignore")
            except IOError:
                continue
            if "evse_manager.py" in cmd:
                continue
            low = cmd.lower()
            if "python" in low and (".py" in low):
                strays.append((pid, cmd.strip()))
    except Exception:
        pass
    if strays:
        print("")
        print("    other python processes (check for an old charge script):")
        for pid, cmd in strays[:12]:
            print("      pid %s  %s" % (pid, cmd[:66]))

    # 3. the wallbox gateway (read only - safe while the service runs)
    print("")
    print("    asking the wallbox gateway for its status (read only)...")
    gw = Gateway(EVSE_IP, EVSE_HTTP_PORT, 2.0)
    try:
        st = gw._request("GET", "/api/status")
        cfg = gw._request("GET", "/api/settings")
        ctl = st.get("controller") or {}
        evse = st.get("evse") or {}
        print("    OK  gateway firmware %s answered." % st.get("fw"))
        print("        EVSE board      : %s (EVRACING rev %s)"
              % ("answering" if st.get("link") else "NOT answering",
                 evse.get("fw")))
        print("        state           : %s - %s"
              % (st.get("state"), st.get("reason")))
        print("        controller      : %s, last command %s s ago from %s"
              % ("active" if ctl.get("alive") else "silent", ctl.get("age"),
                 ctl.get("ip")))
        print("        page switch     : %s" % ("on" if st.get("switch") else
                                                 "OFF - charging disabled"))
        if st.get("meter_energy") is not None:
            print("        meter counter   : %.2f kWh (as the gateway shows it)"
                  % (st["meter_energy"] / 1000.0))
        print("        watchdog        : %s s" % cfg.get("ctrlTimeout"))
        print("        max current     : %s A" % cfg.get("maxCurrent"))
        if (cfg.get("maxCurrent") or 0) < ABS_MAX_A:
            print("        WARNING: mode C cannot reach %.0f A" % ABS_MAX_A)
    except LinkError as exc:
        print("    FAIL %s" % exc)
        print("         Check the gateway is powered and reachable at %s"
              % EVSE_IP)

    # 4. the meter (only when the service is not running - it owns the port)
    print("")
    if not METER_DEVICE:
        print("    no energy meter configured (METER_DEVICE is empty)")
    elif others:
        print("    skipping the meter test: the running service owns %s"
              % METER_DEVICE)
    else:
        meter = EnergyMeter(log, ModbusSerial(METER_DEVICE, METER_BAUD,
                                              METER_UNIT, METER_TIMEOUT_S))
        reading = meter.read()
        kwh = meter.read_energy() if reading else None
        meter.close()
        if reading:
            print("    OK  meter: %.1f V  %.2f A  %.0f W" % reading)
            if kwh is not None:
                print("        counter: %.2f kWh" % kwh)
        else:
            print("    FAIL meter did not answer on %s (address %d, %d baud)"
                  % (METER_DEVICE, METER_UNIT, METER_BAUD))
    print("=" * 62)
    return 0


def probe(log):
    """Print the gateway status once, exactly as it sends it."""
    gw = Gateway(EVSE_IP, EVSE_HTTP_PORT, 2.0)
    try:
        print(json.dumps(gw._request("GET", "/api/status"), indent=2,
                         sort_keys=True))
    except LinkError as exc:
        print("FAIL %s" % exc)
        return 1
    return 0


def acquire_lock(log):
    """
    Refuse to run twice. Two supervised copies would each hold a Modbus
    connection and each think the other's writes were interference.
    Returns the open file object (must be kept alive) or None.
    """
    try:
        import fcntl
    except ImportError:
        return True
    try:
        os.makedirs(BASE_DIR, exist_ok=True)
        handle = open(LOCK_FILE, "w")
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        handle.write("%d\n" % os.getpid())
        handle.flush()
        return handle
    except IOError:
        log.error("another instance is already running - exiting")
        return None
    except Exception:
        return True


def make_logger(verbose):
    log = logging.getLogger("evse")
    log.setLevel(logging.DEBUG if verbose else logging.INFO)
    fmt = logging.Formatter("%(asctime)s %(levelname)-7s %(message)s",
                            "%Y-%m-%d %H:%M:%S")
    stream = logging.StreamHandler(sys.stdout)
    stream.setFormatter(fmt)
    log.addHandler(stream)
    try:
        os.makedirs(BASE_DIR, exist_ok=True)
        rotating = logging.handlers.RotatingFileHandler(
            LOG_FILE, maxBytes=1024 * 1024, backupCount=3)
        rotating.setFormatter(fmt)
        log.addHandler(rotating)
    except Exception:
        pass
    return log


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dump", action="store_true",
                        help="print all Victron D-Bus paths and exit")
    parser.add_argument("--probe", action="store_true",
                        help="print the wallbox gateway status once and exit")
    parser.add_argument("--doctor", action="store_true",
                        help="check for duplicate instances, the gateway and "
                             "the meter")
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()

    log = make_logger(args.verbose)

    applied, retired, cfg_err = load_config()
    apply_min_current()
    if cfg_err:
        log.warning("%s", cfg_err)
    if applied:
        log.info("config.json applied: %s", ", ".join(sorted(applied)))
    elif os.path.exists(CONFIG_FILE):
        log.warning("config.json exists but set nothing")
    else:
        log.warning("no config.json - using built-in defaults")
    if retired:
        log.info("config.json: no longer used, ignored: %s",
                 ", ".join(sorted(retired)))

    if args.probe:
        return probe(log)
    if args.doctor:
        return doctor(log, applied, retired, cfg_err)

    ve = VeDbus(log)

    if args.dump:
        print(json.dumps(ve.dump(), indent=2, sort_keys=True))
        return 0

    lock = acquire_lock(log)
    if lock is None:
        # Sleep before exiting so a supervisor cannot spin on this.
        time.sleep(30)
        return 1

    ctl = Controller(ve, log)
    ctl.mqtt = HaMqtt(ctl, log)
    ctl.mqtt.start()

    from gi.repository import GLib

    loop = GLib.MainLoop()

    def refresh():
        try:
            ve.refresh()
        except Exception as exc:
            log.warning("dbus refresh: %s", exc)
        return True

    GLib.timeout_add_seconds(int(DBUS_REFRESH_S), refresh)

    worker = threading.Thread(target=ctl.run, name="control", daemon=True)
    worker.start()

    def supervise():
        """
        Notice our own failures instead of pretending nothing happened: the
        control thread dying (exit, so daemontools restarts us cleanly) and
        the MQTT link dropping (force a reconnect).
        """
        if not worker.is_alive() and not ctl.stop_flag.is_set():
            log.error("control thread died - exiting so the supervisor "
                      "restarts us")
            record_incident("control thread died, restarting service")
            GLib.idle_add(loop.quit)
            return False
        try:
            if ctl.mqtt:
                ctl.mqtt.watchdog()
        except Exception as exc:
            log.warning("mqtt watchdog: %s", exc)
        return True

    GLib.timeout_add_seconds(15, supervise)

    log.info("evse_manager %s started (mode %s, enabled=%s, wallbox %s)",
             VERSION, ctl.mode, ctl.enabled, EVSE_IP)
    record_incident("service started (version %s, mode %s)"
                    % (VERSION, ctl.mode))
    if PILOT_CAL_A:
        log.info("pilot calibration: +%.2f A added to every current sent to "
                 "the wallbox (old method - set PILOT_CAL_A to 0 in config.json "
                 "to use the learned shortfall instead)", PILOT_CAL_A)
    else:
        log.info("car shortfall %.2f A (mode A asks for this much more; "
                 "learned while charging)", ctl.gap_a)

    def _quit(signum, frame):
        log.info("signal %s, shutting down", signum)
        GLib.idle_add(loop.quit)

    signal.signal(signal.SIGTERM, _quit)
    signal.signal(signal.SIGINT, _quit)

    try:
        loop.run()
    except KeyboardInterrupt:
        pass
    finally:
        ctl.stop_flag.set()
        worker.join(timeout=10)
        ctl.shutdown()
    return 0


def guarded_main():
    """
    A supervised service that exits immediately on an exception gets respawned
    about once a second. If startup fails, log it and idle before exiting so
    the restart rate stays low no matter how the supervisor is configured.
    """
    try:
        return main()
    except SystemExit:
        raise
    except KeyboardInterrupt:
        return 0
    except Exception:
        try:
            log = logging.getLogger("evse")
            if not log.handlers:
                log = make_logger(False)
            log.exception("fatal error during startup - idling before exit")
        except Exception:
            import traceback
            traceback.print_exc()
        time.sleep(30)
        return 1


if __name__ == "__main__":
    sys.exit(guarded_main())
