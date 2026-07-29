#!/bin/sh
set -eu

BASE=/data/openpilot
cd "$BASE"

cat > "$BASE/install_dnga_brake_debug_hook.py" <<'PYHOOK'
#!/usr/bin/env python3
from __future__ import print_function

import os
import py_compile
import shutil

PATH = "/data/openpilot/selfdrive/car/dnga/carcontroller.py"
BACKUP = PATH + ".before_brake_logger"
IMPORT_MARKER = "# DNGA_BRAKE_DEBUG_IMPORTS"
INIT_MARKER = "# DNGA_BRAKE_DEBUG_INIT_BEGIN"
HOOK_MARKER = "# DNGA_BRAKE_DEBUG_HOOK_BEGIN"


def insert_imports(text):
  if IMPORT_MARKER in text:
    return text

  lines = text.splitlines(True)
  index = 0

  if lines and lines[0].startswith("#!"):
    index = 1

  while index < len(lines) and lines[index].startswith("from __future__ import"):
    index += 1

  block = (
    IMPORT_MARKER + "\n"
    "import json\n"
    "import socket\n"
  )
  lines.insert(index, block)
  return "".join(lines)


def insert_init(text):
  if INIT_MARKER in text:
    return text

  update_pos = text.find("\n  def update(")
  if update_pos < 0:
    raise RuntimeError("Could not locate CarController.update().")

  before_update = text[:update_pos]
  target = "    self.brake_active = False"
  init_pos = before_update.rfind(target)

  if init_pos < 0:
    raise RuntimeError("Could not locate self.brake_active initialization.")

  line_end = text.find("\n", init_pos)
  if line_end < 0:
    line_end = len(text)

  block = '''

    # DNGA_BRAKE_DEBUG_INIT_BEGIN
    try:
      self._dnga_brake_debug_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
      self._dnga_brake_debug_sock.setblocking(False)
    except Exception:
      self._dnga_brake_debug_sock = None
    # DNGA_BRAKE_DEBUG_INIT_END'''

  return text[:line_end] + block + text[line_end:]


def insert_hook(text):
  if HOOK_MARKER in text:
    return text

  target = "      brake_amt_for_hud = clip(brake_request, 0.0, 0.60) if decel_req else 0.0"
  pos = text.find(target)

  if pos < 0:
    raise RuntimeError(
      "Could not locate the v2.5c brake_amt_for_hud line. "
      "This installer is intended for the v2.5c brake-only CarController."
    )

  line_end = text.find("\n", pos)
  if line_end < 0:
    line_end = len(text)

  block = r'''

      # DNGA_BRAKE_DEBUG_HOOK_BEGIN
      try:
        _brake_debug = {
          "frame": int(frame),
          "enabled": bool(enabled),
          "active": bool(active),
          "cruiseEnabled": bool(CS.out.cruiseState.enabled),
          "leadArg": bool(lead),
          "vEgo": float(CS.out.vEgo),
          "aEgo": float(CS.out.aEgo),
          "gasPressed": bool(CS.out.gasPressed),
          "brakePressed": bool(CS.out.brakePressed),
          "standstill": bool(CS.out.standstill),
          "cruiseSetSpeed": float(set_speed),
          "actuatorAccel": float(actuators.accel),
          "actuatorSpeed": float(getattr(actuators, "speed", 0.0)),
          "applyAccel": float(apply_accel),
          "applyBrake": float(apply_brake),
          "desSpeed": float(des_speed),
          "baseSpeedForLead": float(base_speed_for_lead),
          "targetSpeedForLead": float(target_speed_for_lead),
          "leadSpeedError": float(lead_speed_error),
          "leadAssistOK": bool(lead_assist_ok),
          "leadAssistBrake": float(lead_assist_brake),
          "overspeed": float(overspeed),
          "overspeedBrake": float(overspeed_brake),
          "brakeRequest": float(brake_request),
          "leadJustAppeared": bool(lead_just_appeared),
          "cutInSoftActive": bool(cut_in_soft_active),
          "cutInBrakeOK": bool(cut_in_brake_ok),
          "leadAssistActive": bool(lead_assist_active),
          "accelGuardOK": bool(accel_guard_ok),
          "brakeAllowed": bool(brake_allowed),
          "brakeReqCounter": int(self.brake_req_counter),
          "brakeReleaseCounter": int(self.brake_release_counter),
          "brakeActive": bool(self.brake_active),
          "decelReq": bool(decel_req),
          "brakeState": int(brake_state),
          "pumpReaction": float(pump_reaction),
          "brakeMag": int(brake_mag),
          "brakeAmtForHud": float(brake_amt_for_hud),
          "blockBrakeUntilFrame": int(self.block_brake_until_frame),
          "framesUntilBrakeAllowed": int(max(0, self.block_brake_until_frame - frame)),
          "cutInSoftUntilFrame": int(getattr(self, "cut_in_soft_until_frame", 0)),
        }

        if self._dnga_brake_debug_sock is not None:
          self._dnga_brake_debug_sock.sendto(
            json.dumps(_brake_debug, separators=(",", ":")).encode("utf-8"),
            ("127.0.0.1", 8062)
          )
      except Exception:
        pass
      # DNGA_BRAKE_DEBUG_HOOK_END'''

  return text[:line_end] + block + text[line_end:]


def main():
  if not os.path.isfile(PATH):
    raise SystemExit("Not found: " + PATH)

  with open(PATH, "r") as f:
    original = f.read()

  if HOOK_MARKER in original and INIT_MARKER in original:
    print("DNGA brake debug hook is already installed.")
    return

  if not os.path.exists(BACKUP):
    shutil.copy2(PATH, BACKUP)
    print("Backup created:", BACKUP)
  else:
    print("Using existing backup:", BACKUP)

  try:
    updated = insert_imports(original)
    updated = insert_init(updated)
    updated = insert_hook(updated)

    with open(PATH, "w") as f:
      f.write(updated)

    py_compile.compile(PATH, doraise=True)
  except Exception as exc:
    shutil.copy2(BACKUP, PATH)
    raise SystemExit("Install failed; original restored: %s" % exc)

  print("Installed DNGA v2.5c brake debug hook.")
  print("CarController compile passed.")


if __name__ == "__main__":
  main()
PYHOOK

cat > "$BASE/dnga_brake_activation_logger.py" <<'PYLOGGER'
#!/usr/bin/env python3
from __future__ import print_function

import csv
import gzip
import json
import os
import signal
import socket
import time
from datetime import datetime

import cereal.messaging as messaging
from common.realtime import Ratekeeper

try:
  from cereal.services import service_list
except Exception:
  service_list = {}

RATE_HZ = 20
DEBUG_PORT = 8062
OUTPUT_DIR = "/data/openpilot"
CAN_IDS = (0x271, 0x273)
STOP_REQUESTED = False


def request_stop(signum, frame):
  global STOP_REQUESTED
  STOP_REQUESTED = True


def get_path(obj, path, default=None):
  try:
    value = obj
    for part in path.split("."):
      value = getattr(value, part)
    return value
  except Exception:
    return default


def to_float(value, default=float("nan")):
  try:
    return float(value)
  except Exception:
    return default


def to_bool(value, default=False):
  try:
    return bool(value)
  except Exception:
    return default


def enum_text(value):
  try:
    return str(value)
  except Exception:
    return ""


def list_value(values, index, default=float("nan")):
  try:
    size = len(values)
    if size == 0:
      return default
    if index < 0:
      index = size + index
    index = max(0, min(index, size - 1))
    return to_float(values[index], default)
  except Exception:
    return default


def hex_data(data):
  try:
    return bytes(data).hex().upper()
  except Exception:
    try:
      return "".join("%02X" % ord(x) for x in data)
    except Exception:
      return ""


def available_services():
  required = ["carState", "carControl", "controlsState"]
  optional = ["radarState", "longitudinalPlan"]

  if not service_list:
    return required + optional

  names = set(service_list.keys())
  result = [name for name in required if name in names]
  result += [name for name in optional if name in names]

  missing = [name for name in required if name not in result]
  if missing:
    raise RuntimeError("Missing required cereal services: " + ", ".join(missing))

  return result


def open_sub_socket(name):
  try:
    return messaging.sub_sock(name, conflate=False)
  except TypeError:
    return messaging.sub_sock(name)


def receive_nonblocking(sock):
  if hasattr(messaging, "recv_one_or_none"):
    return messaging.recv_one_or_none(sock)

  try:
    return messaging.recv_one(sock)
  except Exception:
    return None


def drain_can(sock, field_name, cache):
  event_count = 0
  frame_count = 0

  while True:
    msg = receive_nonblocking(sock)
    if msg is None:
      break

    event_count += 1

    try:
      frames = getattr(msg, field_name)
    except Exception:
      continue

    now = time.monotonic()

    for frame in frames:
      try:
        address = int(frame.address)
        bus = int(frame.src)
      except Exception:
        continue

      if address not in CAN_IDS:
        continue

      cache[address] = {
        "hex": hex_data(frame.dat),
        "bus": bus,
        "time": now,
      }
      frame_count += 1

  return event_count, frame_count


def drain_debug(sock, latest):
  count = 0

  while True:
    try:
      payload, _ = sock.recvfrom(16384)
    except Exception:
      break

    try:
      if not isinstance(payload, str):
        payload = payload.decode("utf-8", "replace")
      packet = json.loads(payload)
      if isinstance(packet, dict):
        latest.clear()
        latest.update(packet)
        latest["_receivedTime"] = time.monotonic()
        count += 1
    except Exception:
      pass

  return count


def can_columns(prefix, cache, now):
  result = {}
  for address in CAN_IDS:
    key = "%s%03X" % (prefix, address)
    item = cache.get(address)
    if item is None:
      result[key] = ""
      result[key + "Bus"] = ""
      result[key + "AgeMs"] = ""
    else:
      result[key] = item["hex"]
      result[key + "Bus"] = item["bus"]
      result[key + "AgeMs"] = (now - item["time"]) * 1000.0
  return result


def field_names():
  fields = [
    "t", "wallTime", "row", "services",
    "debugAlive", "debugAgeMs", "debugPackets",
    "rxEvents", "rxFrames", "txEvents", "txFrames",

    "vEgo", "vEgoKph", "aEgo", "gas", "gasPressed",
    "brake", "brakePressed", "standstill",
    "cruiseEnabled", "cruiseAvailable", "cruiseStandstill",
    "cruiseSpeed", "cruiseSpeedKph", "cruiseSpeedClusterKph",

    "carControlEnabled", "carControlActive",
    "actAccel", "actSpeed", "actSpeedKph", "actGas", "actBrake",

    "controlsEnabled", "controlsActive", "controlsATarget",
    "controlsVPid", "controlsVCruise", "longControlState",
    "forceDecel",

    "planHasLead", "planSource", "planFcw",
    "planSpeed0", "planSpeed5", "planSpeedEnd",
    "planAccel0", "planAccel5", "planAccelEnd",

    "leadStatus", "leadDRel", "leadYRel", "leadVRel",
    "leadVLead", "leadVLeadK", "leadALeadK", "leadALeadTau", "leadFcw",

    "dbgFrame", "dbgEnabled", "dbgActive", "dbgCruiseEnabled", "dbgLeadArg",
    "dbgVEgo", "dbgAEgo", "dbgGasPressed", "dbgBrakePressed", "dbgStandstill",
    "dbgCruiseSetSpeed", "dbgActuatorAccel", "dbgActuatorSpeed",
    "dbgApplyAccel", "dbgApplyBrake", "dbgDesSpeed",
    "dbgBaseSpeedForLead", "dbgTargetSpeedForLead", "dbgLeadSpeedError",
    "dbgLeadAssistOK", "dbgLeadAssistBrake",
    "dbgOverspeed", "dbgOverspeedBrake", "dbgBrakeRequest",
    "dbgLeadJustAppeared", "dbgCutInSoftActive", "dbgCutInBrakeOK",
    "dbgLeadAssistActive", "dbgAccelGuardOK", "dbgBrakeAllowed",
    "dbgBrakeReqCounter", "dbgBrakeReleaseCounter", "dbgBrakeActive",
    "dbgDecelReq", "dbgBrakeState", "dbgPumpReaction", "dbgBrakeMag",
    "dbgBrakeAmtForHud", "dbgBlockBrakeUntilFrame",
    "dbgFramesUntilBrakeAllowed", "dbgCutInSoftUntilFrame",
  ]

  for prefix in ("rx", "tx"):
    for address in CAN_IDS:
      key = "%s%03X" % (prefix, address)
      fields += [key, key + "Bus", key + "AgeMs"]

  return fields


def debug_value(debug, name, default=""):
  return debug.get(name, default)


def main():
  signal.signal(signal.SIGTERM, request_stop)
  signal.signal(signal.SIGINT, request_stop)

  services = available_services()
  sm = messaging.SubMaster(services)

  can_sock = open_sub_socket("can")
  sendcan_sock = open_sub_socket("sendcan")

  debug_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  debug_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
  debug_sock.bind(("127.0.0.1", DEBUG_PORT))
  debug_sock.setblocking(False)

  stamp = datetime.now().strftime("%m%d_%H%M%S")
  path = os.path.join(OUTPUT_DIR, "dnga_brake_activation_%s.csv.gz" % stamp)

  debug = {}
  rx_cache = {}
  tx_cache = {}
  start = time.monotonic()
  last_print = start
  row_number = 0
  fields = field_names()
  rk = Ratekeeper(RATE_HZ)

  print("DNGA v2.5c brake activation logger")
  print("Services: " + ", ".join(services))
  print("Output: " + path)
  print("Debug UDP port: %d" % DEBUG_PORT)

  try:
    with gzip.open(path, "wt", newline="") as output:
      writer = csv.DictWriter(output, fieldnames=fields, extrasaction="ignore")
      writer.writeheader()

      while not STOP_REQUESTED:
        now = time.monotonic()
        sm.update(0)

        rx_events, rx_frames = drain_can(can_sock, "can", rx_cache)
        tx_events, tx_frames = drain_can(sendcan_sock, "sendcan", tx_cache)
        debug_packets = drain_debug(debug_sock, debug)

        cs = sm["carState"]
        cc = sm["carControl"]
        controls = sm["controlsState"]
        actuators = get_path(cc, "actuators", None)
        cruise = get_path(cs, "cruiseState", None)

        radar = sm["radarState"] if "radarState" in services else None
        lead_one = get_path(radar, "leadOne", None)
        plan = sm["longitudinalPlan"] if "longitudinalPlan" in services else None

        debug_time = to_float(debug.get("_receivedTime", 0.0), 0.0)
        debug_age_ms = (now - debug_time) * 1000.0 if debug_time > 0.0 else float("nan")
        debug_alive = debug_time > 0.0 and debug_age_ms < 300.0

        plan_speeds = get_path(plan, "speeds", [])
        plan_accels = get_path(plan, "accels", [])

        row = {
          "t": now - start,
          "wallTime": datetime.now().isoformat(),
          "row": row_number,
          "services": ",".join(services),
          "debugAlive": debug_alive,
          "debugAgeMs": debug_age_ms,
          "debugPackets": debug_packets,
          "rxEvents": rx_events,
          "rxFrames": rx_frames,
          "txEvents": tx_events,
          "txFrames": tx_frames,

          "vEgo": to_float(get_path(cs, "vEgo", 0.0), 0.0),
          "vEgoKph": to_float(get_path(cs, "vEgo", 0.0), 0.0) * 3.6,
          "aEgo": to_float(get_path(cs, "aEgo", 0.0), 0.0),
          "gas": to_float(get_path(cs, "gas", float("nan"))),
          "gasPressed": to_bool(get_path(cs, "gasPressed", False)),
          "brake": to_float(get_path(cs, "brake", float("nan"))),
          "brakePressed": to_bool(get_path(cs, "brakePressed", False)),
          "standstill": to_bool(get_path(cs, "standstill", False)),
          "cruiseEnabled": to_bool(get_path(cruise, "enabled", False)),
          "cruiseAvailable": to_bool(get_path(cruise, "available", False)),
          "cruiseStandstill": to_bool(get_path(cruise, "standstill", False)),
          "cruiseSpeed": to_float(get_path(cruise, "speed", float("nan"))),
          "cruiseSpeedKph": to_float(get_path(cruise, "speed", 0.0), 0.0) * 3.6,
          "cruiseSpeedClusterKph": to_float(get_path(cruise, "speedCluster", 0.0), 0.0) * 3.6,

          "carControlEnabled": to_bool(get_path(cc, "enabled", False)),
          "carControlActive": to_bool(get_path(cc, "active", False)),
          "actAccel": to_float(get_path(actuators, "accel", float("nan"))),
          "actSpeed": to_float(get_path(actuators, "speed", float("nan"))),
          "actSpeedKph": to_float(get_path(actuators, "speed", 0.0), 0.0) * 3.6,
          "actGas": to_float(get_path(actuators, "gas", float("nan"))),
          "actBrake": to_float(get_path(actuators, "brake", float("nan"))),

          "controlsEnabled": to_bool(get_path(controls, "enabled", False)),
          "controlsActive": to_bool(get_path(controls, "active", False)),
          "controlsATarget": to_float(get_path(controls, "aTarget", float("nan"))),
          "controlsVPid": to_float(get_path(controls, "vPid", float("nan"))),
          "controlsVCruise": to_float(get_path(controls, "vCruise", float("nan"))),
          "longControlState": enum_text(get_path(controls, "longControlState", "")),
          "forceDecel": to_bool(get_path(controls, "forceDecel", False)),

          "planHasLead": to_bool(get_path(plan, "hasLead", False)),
          "planSource": enum_text(get_path(plan, "longitudinalPlanSource", "")),
          "planFcw": to_bool(get_path(plan, "fcw", False)),
          "planSpeed0": list_value(plan_speeds, 0),
          "planSpeed5": list_value(plan_speeds, 5),
          "planSpeedEnd": list_value(plan_speeds, -1),
          "planAccel0": list_value(plan_accels, 0),
          "planAccel5": list_value(plan_accels, 5),
          "planAccelEnd": list_value(plan_accels, -1),

          "leadStatus": to_bool(get_path(lead_one, "status", False)),
          "leadDRel": to_float(get_path(lead_one, "dRel", float("nan"))),
          "leadYRel": to_float(get_path(lead_one, "yRel", float("nan"))),
          "leadVRel": to_float(get_path(lead_one, "vRel", float("nan"))),
          "leadVLead": to_float(get_path(lead_one, "vLead", float("nan"))),
          "leadVLeadK": to_float(get_path(lead_one, "vLeadK", float("nan"))),
          "leadALeadK": to_float(get_path(lead_one, "aLeadK", float("nan"))),
          "leadALeadTau": to_float(get_path(lead_one, "aLeadTau", float("nan"))),
          "leadFcw": to_bool(get_path(lead_one, "fcw", False)),

          "dbgFrame": debug_value(debug, "frame"),
          "dbgEnabled": debug_value(debug, "enabled"),
          "dbgActive": debug_value(debug, "active"),
          "dbgCruiseEnabled": debug_value(debug, "cruiseEnabled"),
          "dbgLeadArg": debug_value(debug, "leadArg"),
          "dbgVEgo": debug_value(debug, "vEgo"),
          "dbgAEgo": debug_value(debug, "aEgo"),
          "dbgGasPressed": debug_value(debug, "gasPressed"),
          "dbgBrakePressed": debug_value(debug, "brakePressed"),
          "dbgStandstill": debug_value(debug, "standstill"),
          "dbgCruiseSetSpeed": debug_value(debug, "cruiseSetSpeed"),
          "dbgActuatorAccel": debug_value(debug, "actuatorAccel"),
          "dbgActuatorSpeed": debug_value(debug, "actuatorSpeed"),
          "dbgApplyAccel": debug_value(debug, "applyAccel"),
          "dbgApplyBrake": debug_value(debug, "applyBrake"),
          "dbgDesSpeed": debug_value(debug, "desSpeed"),
          "dbgBaseSpeedForLead": debug_value(debug, "baseSpeedForLead"),
          "dbgTargetSpeedForLead": debug_value(debug, "targetSpeedForLead"),
          "dbgLeadSpeedError": debug_value(debug, "leadSpeedError"),
          "dbgLeadAssistOK": debug_value(debug, "leadAssistOK"),
          "dbgLeadAssistBrake": debug_value(debug, "leadAssistBrake"),
          "dbgOverspeed": debug_value(debug, "overspeed"),
          "dbgOverspeedBrake": debug_value(debug, "overspeedBrake"),
          "dbgBrakeRequest": debug_value(debug, "brakeRequest"),
          "dbgLeadJustAppeared": debug_value(debug, "leadJustAppeared"),
          "dbgCutInSoftActive": debug_value(debug, "cutInSoftActive"),
          "dbgCutInBrakeOK": debug_value(debug, "cutInBrakeOK"),
          "dbgLeadAssistActive": debug_value(debug, "leadAssistActive"),
          "dbgAccelGuardOK": debug_value(debug, "accelGuardOK"),
          "dbgBrakeAllowed": debug_value(debug, "brakeAllowed"),
          "dbgBrakeReqCounter": debug_value(debug, "brakeReqCounter"),
          "dbgBrakeReleaseCounter": debug_value(debug, "brakeReleaseCounter"),
          "dbgBrakeActive": debug_value(debug, "brakeActive"),
          "dbgDecelReq": debug_value(debug, "decelReq"),
          "dbgBrakeState": debug_value(debug, "brakeState"),
          "dbgPumpReaction": debug_value(debug, "pumpReaction"),
          "dbgBrakeMag": debug_value(debug, "brakeMag"),
          "dbgBrakeAmtForHud": debug_value(debug, "brakeAmtForHud"),
          "dbgBlockBrakeUntilFrame": debug_value(debug, "blockBrakeUntilFrame"),
          "dbgFramesUntilBrakeAllowed": debug_value(debug, "framesUntilBrakeAllowed"),
          "dbgCutInSoftUntilFrame": debug_value(debug, "cutInSoftUntilFrame"),
        }

        row.update(can_columns("rx", rx_cache, now))
        row.update(can_columns("tx", tx_cache, now))
        writer.writerow(row)

        row_number += 1
        if row_number % 100 == 0:
          output.flush()

        if now - last_print >= 5.0:
          tx271 = tx_cache.get(0x271)
          tx271_text = "none"
          if tx271 is not None and now - tx271["time"] < 1.0:
            tx271_text = "bus%d %s" % (tx271["bus"], tx271["hex"])

          print(
            "%.1fs rows=%d speed=%.1f debug=%s brakeReq=%s allowed=%s active=%s state=%s mag=%s tx271=%s" % (
              now - start,
              row_number,
              row["vEgoKph"],
              "alive" if debug_alive else "MISSING",
              str(row["dbgBrakeRequest"]),
              str(row["dbgBrakeAllowed"]),
              str(row["dbgBrakeActive"]),
              str(row["dbgBrakeState"]),
              str(row["dbgBrakeMag"]),
              tx271_text,
            ),
            flush=True,
          )
          last_print = now

        rk.keep_time()

  finally:
    try:
      debug_sock.close()
    except Exception:
      pass

  print("Logger stopped cleanly.")
  print("Saved %d rows to:" % row_number)
  print(path)


if __name__ == "__main__":
  main()
PYLOGGER

cat > "$BASE/dnga_brake_logger_ctl.sh" <<'SHCTL'
#!/bin/sh

BASE=/data/openpilot
LOGGER="$BASE/dnga_brake_activation_logger.py"
PIDFILE="$BASE/dnga_brake_logger.pid"
STATUSFILE="$BASE/dnga_brake_logger_status.txt"

is_running() {
  [ -f "$PIDFILE" ] || return 1
  PID=$(cat "$PIDFILE" 2>/dev/null)
  [ -n "$PID" ] || return 1
  kill -0 "$PID" 2>/dev/null
}

start_logger() {
  if is_running; then
    echo "Logger is already running. PID: $PID"
    exit 0
  fi

  rm -f "$PIDFILE"

  if [ ! -f "$LOGGER" ]; then
    echo "Logger not found: $LOGGER"
    exit 1
  fi

  cd "$BASE" || exit 1

  nohup env PYTHONPATH="$BASE:$BASE/cereal" \
    python -u "$LOGGER" \
    > "$STATUSFILE" 2>&1 </dev/null &

  PID=$!
  echo "$PID" > "$PIDFILE"
  sleep 2

  if kill -0 "$PID" 2>/dev/null; then
    echo "Logger started. PID: $PID"
    echo "SSH can now be closed safely."
    echo "Status file: $STATUSFILE"
    tail -n 8 "$STATUSFILE"
  else
    echo "Logger failed to start."
    cat "$STATUSFILE"
    rm -f "$PIDFILE"
    exit 1
  fi
}

stop_logger() {
  if ! is_running; then
    echo "Logger is not running."
    rm -f "$PIDFILE"
    exit 0
  fi

  echo "Stopping logger PID $PID..."
  kill -TERM "$PID" 2>/dev/null

  COUNT=0
  while kill -0 "$PID" 2>/dev/null && [ "$COUNT" -lt 10 ]; do
    sleep 1
    COUNT=$((COUNT + 1))
  done

  if kill -0 "$PID" 2>/dev/null; then
    echo "Logger did not stop cleanly; forcing stop."
    kill -KILL "$PID" 2>/dev/null
  fi

  rm -f "$PIDFILE"
  echo "Logger stopped."
  tail -n 12 "$STATUSFILE" 2>/dev/null
  echo
  ls -lht "$BASE"/dnga_brake_activation_*.csv.gz 2>/dev/null | head -n 3
}

status_logger() {
  if is_running; then
    echo "Logger is running. PID: $PID"
  else
    echo "Logger is not running."
  fi
  tail -n 12 "$STATUSFILE" 2>/dev/null
}

case "${1:-status}" in
  start) start_logger ;;
  stop) stop_logger ;;
  restart) stop_logger; start_logger ;;
  status) status_logger ;;
  tail) tail -n 30 "$STATUSFILE" 2>/dev/null ;;
  logs) ls -lht "$BASE"/dnga_brake_activation_*.csv.gz 2>/dev/null | head -n 10 ;;
  *)
    echo "Usage: sh dnga_brake_logger_ctl.sh {start|stop|restart|status|tail|logs}"
    exit 1
    ;;
esac
SHCTL

chmod 755 "$BASE/install_dnga_brake_debug_hook.py" "$BASE/dnga_brake_activation_logger.py" "$BASE/dnga_brake_logger_ctl.sh"

python "$BASE/install_dnga_brake_debug_hook.py"
PYTHONPATH="$BASE:$BASE/cereal" python -m py_compile \
  "$BASE/selfdrive/car/dnga/carcontroller.py" \
  "$BASE/dnga_brake_activation_logger.py"
sh -n "$BASE/dnga_brake_logger_ctl.sh"

echo
echo "DNGA brake logger installed successfully."
echo "Reboot before starting the logger."
echo "After reboot: sh /data/openpilot/dnga_brake_logger_ctl.sh start"
