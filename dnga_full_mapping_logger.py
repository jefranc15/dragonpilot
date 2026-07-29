#!/usr/bin/env python3
from __future__ import print_function

import csv
import gzip
import json
import os
import signal
import time
from collections import deque
from datetime import datetime

import cereal.messaging as messaging

try:
  from cereal.services import service_list
except Exception:
  service_list = {}

RATE_HZ = 20.0
PREBUFFER_SECONDS = 8.0
DEFAULT_POST_SECONDS = 12.0
MAX_PREBUFFER_FRAMES = 100000

COMMAND_FILE = "/data/openpilot/dnga_full_mapping_commands.csv"

SERVICES_WANTED = [
  "carState",
  "carControl",
  "controlsState",
  "radarState",
  "longitudinalPlan",
  "liveParameters",
]

KNOWN_IDS = [0x191, 0x271, 0x273, 0x274, 0x2E4, 0x2E6, 0x412, 0x520]
HUD_IDS = [0x273, 0x274, 0x2E4, 0x2E6, 0x412, 0x520]

STOP = False


def stop_handler(signum, frame):
  global STOP
  STOP = True


def available_services():
  if not service_list:
    return ["carState", "carControl", "controlsState",
            "radarState", "liveParameters"]
  result = []
  for name in SERVICES_WANTED:
    try:
      if name in service_list:
        result.append(name)
    except Exception:
      pass
  return result


def get_path(obj, path, default=None):
  try:
    value = obj
    for name in path.split("."):
      value = getattr(value, name)
    return value
  except Exception:
    return default


def first_value(obj, paths, default=None):
  for path in paths:
    value = get_path(obj, path, None)
    if value is not None:
      return value
  return default


def as_float(value, default=float("nan")):
  try:
    return float(value)
  except Exception:
    return default


def as_bool(value, default=False):
  try:
    return bool(value)
  except Exception:
    return default


def as_text(value, default=""):
  try:
    return str(value)
  except Exception:
    return default


def sm_get(sm, service):
  try:
    return sm[service]
  except Exception:
    return None


def sm_alive(sm, service):
  try:
    return bool(sm.alive[service])
  except Exception:
    return False


def open_sock(service):
  try:
    return messaging.sub_sock(service, conflate=False)
  except TypeError:
    return messaging.sub_sock(service)


def recv_none(sock):
  try:
    return messaging.recv_one_or_none(sock)
  except Exception:
    return None


def raw_bytes(data):
  try:
    return bytes(data)
  except Exception:
    try:
      return bytes(bytearray(ord(x) for x in data))
    except Exception:
      return b""


def drain(sock, field_name, direction, start_time, cache):
  events = 0
  output = []

  while True:
    msg = recv_none(sock)
    if msg is None:
      break

    events += 1

    try:
      frames = getattr(msg, field_name)
    except Exception:
      continue

    mono = time.monotonic()
    relative = mono - start_time
    wall = datetime.now().isoformat()

    for frame in frames:
      try:
        address = int(frame.address)
        bus = int(frame.src)
        data = raw_bytes(frame.dat)
      except Exception:
        continue

      item = {
        "t": relative,
        "mono": mono,
        "wallTime": wall,
        "direction": direction,
        "address": address,
        "addressHex": "0x%03X" % address,
        "bus": bus,
        "dlc": len(data),
        "data": data.hex().upper(),
        "raw": data,
      }
      output.append(item)
      cache[(address, bus)] = item

  return events, output


def latest_address(cache, address):
  newest = None
  for (candidate, bus), item in cache.items():
    if candidate != address:
      continue
    if newest is None or item["mono"] > newest["mono"]:
      newest = item
  return newest


def signed_byte(value):
  return value - 256 if value >= 128 else value


def dnga_checksum(address, data7):
  return (address + 8 + 1 + 2 + sum(data7)) & 0xFF


def decode_271(item, prefix):
  result = {
    prefix + "271Raw": "",
    prefix + "271Bus": "",
    prefix + "271AgeMs": "",
    prefix + "271State": "",
    prefix + "271StateName": "",
    prefix + "271Active": False,
    prefix + "271PumpRaw": "",
    prefix + "271Pump": "",
    prefix + "271Magnitude": "",
    prefix + "271Counter": "",
    prefix + "271ChecksumValid": "",
  }

  if item is None:
    return result

  result[prefix + "271Raw"] = item["data"]
  result[prefix + "271Bus"] = item["bus"]
  result[prefix + "271AgeMs"] = (
    time.monotonic() - item["mono"]) * 1000.0

  try:
    data = bytes.fromhex(item["data"])
    if len(data) != 8:
      return result

    state = int(data[1])
    names = {
      0x00: "disabled",
      0x01: "ready_no_brake",
      0x21: "active_brake",
      0x30: "possible_hold_30",
      0x31: "possible_hold_31",
    }

    result[prefix + "271State"] = state
    result[prefix + "271StateName"] = names.get(
      state, "unknown_%02X" % state)
    result[prefix + "271Active"] = state in (0x21, 0x30, 0x31)
    result[prefix + "271PumpRaw"] = signed_byte(int(data[3]))
    result[prefix + "271Pump"] = (
      result[prefix + "271PumpRaw"] / 10.0)
    result[prefix + "271Magnitude"] = (
      (int(data[4]) << 8) | int(data[5]))
    result[prefix + "271Counter"] = (
      (int(data[6]) >> 2) & 0x07)
    result[prefix + "271ChecksumValid"] = (
      int(data[7]) == dnga_checksum(0x271, list(data[:-1])))
  except Exception:
    pass

  return result


def decode_273(item, prefix):
  result = {
    prefix + "273Raw": "",
    prefix + "273Bus": "",
    prefix + "273AgeMs": "",
    prefix + "273AccCmdKph": "",
    prefix + "273Byte0": "",
    prefix + "273Byte1": "",
    prefix + "273Byte4": "",
    prefix + "273Byte5": "",
    prefix + "273Byte6": "",
    prefix + "273ChecksumValid": "",
  }

  if item is None:
    return result

  result[prefix + "273Raw"] = item["data"]
  result[prefix + "273Bus"] = item["bus"]
  result[prefix + "273AgeMs"] = (
    time.monotonic() - item["mono"]) * 1000.0

  try:
    data = bytes.fromhex(item["data"])
    if len(data) != 8:
      return result

    result[prefix + "273AccCmdKph"] = (
      ((int(data[2]) << 8) | int(data[3])) * 0.01)
    result[prefix + "273Byte0"] = int(data[0])
    result[prefix + "273Byte1"] = int(data[1])
    result[prefix + "273Byte4"] = int(data[4])
    result[prefix + "273Byte5"] = int(data[5])
    result[prefix + "273Byte6"] = int(data[6])
    result[prefix + "273ChecksumValid"] = (
      int(data[7]) == dnga_checksum(0x273, list(data[:-1])))
  except Exception:
    pass

  return result


def selected_ids(prefix, cache):
  now = time.monotonic()
  result = {}

  for address in KNOWN_IDS:
    key = prefix + "%03X" % address
    item = latest_address(cache, address)

    if item is None:
      result[key] = ""
      result[key + "Bus"] = ""
      result[key + "AgeMs"] = ""
    else:
      result[key] = item["data"]
      result[key + "Bus"] = item["bus"]
      result[key + "AgeMs"] = (now - item["mono"]) * 1000.0

  return result


def lead_values(radar):
  lead = get_path(radar, "leadOne", None)
  return {
    "leadStatus": as_bool(get_path(lead, "status", False)),
    "leadDRel": as_float(get_path(lead, "dRel", float("nan"))),
    "leadYRel": as_float(get_path(lead, "yRel", float("nan"))),
    "leadVRel": as_float(get_path(lead, "vRel", float("nan"))),
    "leadVLead": as_float(get_path(lead, "vLead", float("nan"))),
    "leadVLeadK": as_float(get_path(lead, "vLeadK", float("nan"))),
    "leadALeadK": as_float(get_path(lead, "aLeadK", float("nan"))),
    "leadModelProb": as_float(
      get_path(lead, "modelProb", float("nan"))),
    "leadFcw": as_bool(get_path(lead, "fcw", False)),
  }


def plan_values(plan):
  output = {
    "planSource": as_text(
      get_path(plan, "longitudinalPlanSource", "")),
    "planShouldStop": as_bool(
      get_path(plan, "shouldStop", False)),
    "planSpeed0": float("nan"),
    "planSpeedEnd": float("nan"),
    "planAccel0": float("nan"),
    "planAccelEnd": float("nan"),
  }

  try:
    speeds = get_path(plan, "speeds", [])
    output["planSpeed0"] = float(speeds[0])
    output["planSpeedEnd"] = float(speeds[-1])
  except Exception:
    pass

  try:
    accels = get_path(plan, "accels", [])
    output["planAccel0"] = float(accels[0])
    output["planAccelEnd"] = float(accels[-1])
  except Exception:
    pass

  return output


def wheel_values(car_state):
  wheels = get_path(car_state, "wheelSpeeds", None)
  return {
    "wheelFL": as_float(get_path(wheels, "fl", float("nan"))),
    "wheelFR": as_float(get_path(wheels, "fr", float("nan"))),
    "wheelRL": as_float(get_path(wheels, "rl", float("nan"))),
    "wheelRR": as_float(get_path(wheels, "rr", float("nan"))),
  }


def button_values(car_state):
  result = []

  try:
    for event in get_path(car_state, "buttonEvents", []):
      try:
        event_type = event.type.raw
      except Exception:
        event_type = as_text(get_path(event, "type", ""))

      result.append({
        "type": event_type,
        "pressed": as_bool(get_path(event, "pressed", False)),
      })
  except Exception:
    pass

  return result


def button_signature(events):
  parts = []
  for event in events:
    parts.append("%s:%s" % (
      event.get("type", ""),
      "down" if event.get("pressed", False) else "up",
    ))
  return "|".join(parts)


def normalize_hud(address, data):
  value = bytearray(data)

  # Mask known fast-changing speed/counter/checksum bytes.
  ignored = {
    0x273: (2, 3, 6, 7),
    0x271: (4, 5, 6, 7),
    0x274: (6, 7),
    0x2E4: (6, 7),
    0x2E6: (6, 7),
    0x412: (6, 7),
    0x520: (7,),
  }

  for index in ignored.get(address, ()):
    if index < len(value):
      value[index] = 0

  return bytes(value)


def bit_changes(previous, current):
  length = max(len(previous), len(current))
  old = previous.ljust(length, b"\x00")
  new = current.ljust(length, b"\x00")
  output = []

  for byte_index in range(length):
    xor_value = old[byte_index] ^ new[byte_index]
    for bit_index in range(8):
      if xor_value & (1 << bit_index):
        old_bit = (old[byte_index] >> bit_index) & 1
        new_bit = (new[byte_index] >> bit_index) & 1
        output.append("B%d.b%d:%d>%d" % (
          byte_index, bit_index, old_bit, new_bit))

  return ";".join(output)


def classify_brake(brake_pressed, brake_lights, cruise_enabled,
                   rx271_active, a_ego):
  if brake_pressed:
    return "manual_brake"
  if cruise_enabled and (
      rx271_active or brake_lights or a_ego <= -0.20):
    return "stock_acc_brake"
  if rx271_active:
    return "stock_ecb_active"
  if a_ego <= -0.50:
    return "vehicle_decelerating"
  return "none"


def poll_commands(offset):
  commands = []

  try:
    if not os.path.exists(COMMAND_FILE):
      return offset, commands

    with open(COMMAND_FILE, "r") as stream:
      stream.seek(offset)
      for line in stream:
        line = line.strip()
        if not line:
          continue

        parts = line.split(",", 2)
        action = parts[0].strip()

        if action == "capture" and len(parts) >= 3:
          try:
            duration = float(parts[1])
          except Exception:
            duration = DEFAULT_POST_SECONDS
          commands.append(("capture", duration, parts[2].strip()))

        elif action == "mark" and len(parts) >= 2:
          label = ",".join(parts[1:]).strip()
          commands.append(("capture", DEFAULT_POST_SECONDS, label))

      offset = stream.tell()
  except Exception:
    pass

  return offset, commands


def add_reason(reasons, reason):
  if reason and reason not in reasons:
    reasons.append(reason)


SUMMARY_FIELDS = [
  "t", "wallTime", "row",
  "eventWindow", "eventId", "eventReasons",
  "brakeClassification",
  "rxEvents", "rxFrames", "txEvents", "txFrames",
  "rawFramesWritten", "changeRowsWritten",

  "carStateAlive", "carControlAlive",
  "controlsStateAlive", "radarStateAlive",
  "longitudinalPlanAlive",

  "vEgo", "vEgoKph", "vEgoRaw", "aEgo", "estimatedJerk",
  "standstill", "gearShifter",
  "engineRpm", "engineSpeedRpm", "motorRpm", "transmissionRpm",

  "gas", "gasPressed", "gasDelta",
  "brake", "brakePressed", "brakeLights",

  "wheelFL", "wheelFR", "wheelRL", "wheelRR",

  "cruiseAvailable", "cruiseEnabled",
  "cruiseStandstill", "cruiseSpeedKph",

  "buttonEventsJson", "buttonEventSignature",

  "carControlEnabled", "carControlActive",
  "actAccel", "actGas", "actBrake", "actSpeed", "actSteer",

  "controlsEnabled", "controlsActive",
  "controlsATarget", "controlsVCruise",
  "longControlState", "forceDecel",

  "planSource", "planShouldStop",
  "planSpeed0", "planSpeedEnd",
  "planAccel0", "planAccelEnd",

  "leadStatus", "leadDRel", "leadYRel", "leadVRel",
  "leadVLead", "leadVLeadK", "leadALeadK",
  "leadModelProb", "leadFcw",

  "steeringAngleDeg", "steeringRateDeg",
  "steeringTorque", "steeringTorqueEps",
  "steeringPressed", "steerRatioLive",

  "opLongUnexpected",
]

for prefix in ("rx", "tx"):
  SUMMARY_FIELDS.extend([
    prefix + "271Raw", prefix + "271Bus", prefix + "271AgeMs",
    prefix + "271State", prefix + "271StateName",
    prefix + "271Active", prefix + "271PumpRaw",
    prefix + "271Pump", prefix + "271Magnitude",
    prefix + "271Counter", prefix + "271ChecksumValid",

    prefix + "273Raw", prefix + "273Bus", prefix + "273AgeMs",
    prefix + "273AccCmdKph", prefix + "273Byte0",
    prefix + "273Byte1", prefix + "273Byte4",
    prefix + "273Byte5", prefix + "273Byte6",
    prefix + "273ChecksumValid",
  ])

  for address in KNOWN_IDS:
    key = prefix + "%03X" % address
    SUMMARY_FIELDS.extend([key, key + "Bus", key + "AgeMs"])


RAW_FIELDS = [
  "sequence", "t", "wallTime",
  "eventId", "eventReasons",
  "direction", "address", "addressHex",
  "bus", "dlc", "data",
]

CHANGE_FIELDS = [
  "sequence", "t", "wallTime",
  "eventId", "eventReasons",
  "direction", "address", "addressHex", "bus",
  "previousData", "currentData", "xorHex", "changedBits",
]


def write_raw(writer, frame, event_id, reasons, sequence):
  writer.writerow({
    "sequence": sequence,
    "t": frame["t"],
    "wallTime": frame["wallTime"],
    "eventId": event_id,
    "eventReasons": reasons,
    "direction": frame["direction"],
    "address": frame["address"],
    "addressHex": frame["addressHex"],
    "bus": frame["bus"],
    "dlc": frame["dlc"],
    "data": frame["data"],
  })


def write_change(writer, frame, previous, event_id, reasons, sequence):
  current = frame["raw"]
  length = max(len(previous), len(current))
  old = previous.ljust(length, b"\x00")
  new = current.ljust(length, b"\x00")
  xor_value = bytes(old[i] ^ new[i] for i in range(length))

  writer.writerow({
    "sequence": sequence,
    "t": frame["t"],
    "wallTime": frame["wallTime"],
    "eventId": event_id,
    "eventReasons": reasons,
    "direction": frame["direction"],
    "address": frame["address"],
    "addressHex": frame["addressHex"],
    "bus": frame["bus"],
    "previousData": previous.hex().upper(),
    "currentData": current.hex().upper(),
    "xorHex": xor_value.hex().upper(),
    "changedBits": bit_changes(previous, current),
  })


def main():
  signal.signal(signal.SIGINT, stop_handler)
  signal.signal(signal.SIGTERM, stop_handler)

  os.chdir("/data/openpilot")

  services = available_services()
  sm = messaging.SubMaster(services)
  can_sock = open_sock("can")
  sendcan_sock = open_sock("sendcan")

  stamp = datetime.now().strftime("%m%d_%H%M%S")
  summary_path = (
    "/data/openpilot/dnga_full_mapping_summary_%s.csv.gz" % stamp)
  raw_path = (
    "/data/openpilot/dnga_full_mapping_raw_%s.csv.gz" % stamp)
  change_path = (
    "/data/openpilot/dnga_full_mapping_changes_%s.csv.gz" % stamp)

  print("DNGA full mapping logger")
  print("Passive/read-only: no CAN transmission and no car-file changes.")
  print("Keep OP toggle OFF for stock/manual tests.")
  print("Summary:", summary_path)
  print("Raw CAN:", raw_path)
  print("Bit changes:", change_path)
  print("Automatic windows: 8 seconds before, 12 seconds after.")
  print("Use ctl capture for guaranteed button/HUD capture.")

  start = time.monotonic()
  next_sample = start
  last_status = start

  rx_cache = {}
  tx_cache = {}
  prebuffer = deque(maxlen=MAX_PREBUFFER_FRAMES)
  previous_change = {}
  hud_signatures = {}

  row_count = 0
  raw_sequence = 0
  change_sequence = 0
  raw_written = 0
  changes_written = 0

  event_id = 0
  event_reasons = []
  event_until = 0.0
  command_offset = 0

  previous_a_ego = None
  previous_a_time = None
  previous_gas = None
  previous_values = {}
  previous_strong_accel = False
  previous_strong_decel = False

  with gzip.open(summary_path, "wt", newline="") as summary_file, \
       gzip.open(raw_path, "wt", newline="") as raw_file, \
       gzip.open(change_path, "wt", newline="") as change_file:

    summary_writer = csv.DictWriter(
      summary_file, fieldnames=SUMMARY_FIELDS, extrasaction="ignore")
    raw_writer = csv.DictWriter(
      raw_file, fieldnames=RAW_FIELDS, extrasaction="ignore")
    change_writer = csv.DictWriter(
      change_file, fieldnames=CHANGE_FIELDS, extrasaction="ignore")

    summary_writer.writeheader()
    raw_writer.writeheader()
    change_writer.writeheader()

    while not STOP:
      now = time.monotonic()
      was_active = now <= event_until

      try:
        sm.update(0)
      except TypeError:
        sm.update()

      rx_events, rx_frames = drain(
        can_sock, "can", "rx", start, rx_cache)
      tx_events, tx_frames = drain(
        sendcan_sock, "sendcan", "tx", start, tx_cache)
      frames = rx_frames + tx_frames

      if was_active:
        reasons_text = "|".join(event_reasons)
        for frame in frames:
          raw_sequence += 1
          write_raw(raw_writer, frame, event_id, reasons_text, raw_sequence)
          raw_written += 1

          key = (frame["direction"], frame["address"], frame["bus"])
          previous = previous_change.get(key)
          if previous is not None and previous != frame["raw"]:
            change_sequence += 1
            write_change(
              change_writer, frame, previous,
              event_id, reasons_text, change_sequence)
            changes_written += 1
          previous_change[key] = frame["raw"]
      else:
        for frame in frames:
          prebuffer.append(frame)

      car_state = sm_get(sm, "carState")
      car_control = sm_get(sm, "carControl")
      controls = sm_get(sm, "controlsState")
      radar = sm_get(sm, "radarState")
      plan = sm_get(sm, "longitudinalPlan")
      live = sm_get(sm, "liveParameters")

      cruise = get_path(car_state, "cruiseState", None)
      actuators = get_path(car_control, "actuators", None)

      v_ego = as_float(get_path(car_state, "vEgo", 0.0), 0.0)
      a_ego = as_float(get_path(car_state, "aEgo", float("nan")))
      gas = as_float(get_path(car_state, "gas", float("nan")))
      gas_pressed = as_bool(get_path(car_state, "gasPressed", False))
      brake_pressed = as_bool(
        get_path(car_state, "brakePressed", False))
      brake_lights = as_bool(
        get_path(car_state, "brakeLights", False))
      standstill = as_bool(get_path(car_state, "standstill", False))
      gear = as_text(get_path(car_state, "gearShifter", ""))

      cruise_available = as_bool(
        get_path(cruise, "available", False))
      cruise_enabled = as_bool(get_path(cruise, "enabled", False))
      cruise_standstill = as_bool(
        get_path(cruise, "standstill", False))
      cruise_speed = as_float(
        get_path(cruise, "speed", float("nan")))

      buttons = button_values(car_state)
      buttons_text = button_signature(buttons)

      rx271_item = latest_address(rx_cache, 0x271)
      tx271_item = latest_address(tx_cache, 0x271)
      rx273_item = latest_address(rx_cache, 0x273)
      tx273_item = latest_address(tx_cache, 0x273)

      rx271 = decode_271(rx271_item, "rx")
      tx271 = decode_271(tx271_item, "tx")
      rx273 = decode_273(rx273_item, "rx")
      tx273 = decode_273(tx273_item, "tx")

      rx271_active = as_bool(rx271.get("rx271Active", False))
      brake_classification = classify_brake(
        brake_pressed, brake_lights, cruise_enabled,
        rx271_active, a_ego)

      reasons = []

      if brake_classification != "none":
        add_reason(reasons, "brake:" + brake_classification)

      if buttons_text:
        add_reason(reasons, "button:" + buttons_text)

      current_values = {
        "gasPressed": gas_pressed,
        "brakePressed": brake_pressed,
        "brakeLights": brake_lights,
        "cruiseAvailable": cruise_available,
        "cruiseEnabled": cruise_enabled,
        "cruiseStandstill": cruise_standstill,
        "standstill": standstill,
        "gear": gear,
        "rx271Active": rx271_active,
      }

      for name, value in current_values.items():
        if name in previous_values and value != previous_values[name]:
          add_reason(reasons, "%s:%s>%s" % (
            name, previous_values[name], value))

      gas_delta = float("nan")
      if (previous_gas is not None and gas == gas and
          previous_gas == previous_gas):
        gas_delta = gas - previous_gas
        if abs(gas_delta) >= 0.04:
          add_reason(reasons, "gas_delta:%+.3f" % gas_delta)

      strong_accel = a_ego == a_ego and a_ego >= 0.60
      strong_decel = a_ego == a_ego and a_ego <= -0.60

      if strong_accel and not previous_strong_accel:
        add_reason(reasons, "strong_accel")
      if strong_decel and not previous_strong_decel:
        add_reason(reasons, "strong_decel")

      for frame in frames:
        if frame["address"] not in HUD_IDS:
          continue

        key = (frame["direction"], frame["address"], frame["bus"])
        signature = normalize_hud(frame["address"], frame["raw"])
        previous_signature = hud_signatures.get(key)

        if (previous_signature is not None and
            signature != previous_signature):
          add_reason(reasons, "hud_change:%s:%s:bus%d" % (
            frame["direction"], frame["addressHex"], frame["bus"]))

        hud_signatures[key] = signature

      command_offset, commands = poll_commands(command_offset)
      requested_duration = DEFAULT_POST_SECONDS

      for action, duration, label in commands:
        add_reason(reasons, "manual_capture:" + label)
        if duration > requested_duration:
          requested_duration = duration

      if reasons:
        if not was_active:
          event_id += 1
          event_reasons = []
          for reason in reasons:
            add_reason(event_reasons, reason)

          reasons_text = "|".join(event_reasons)

          for frame in prebuffer:
            raw_sequence += 1
            write_raw(
              raw_writer, frame, event_id, reasons_text, raw_sequence)
            raw_written += 1

            key = (frame["direction"], frame["address"], frame["bus"])
            previous = previous_change.get(key)
            if previous is not None and previous != frame["raw"]:
              change_sequence += 1
              write_change(
                change_writer, frame, previous,
                event_id, reasons_text, change_sequence)
              changes_written += 1
            previous_change[key] = frame["raw"]

          prebuffer.clear()
        else:
          for reason in reasons:
            add_reason(event_reasons, reason)

        event_until = max(event_until, now + requested_duration)

      event_active = now <= event_until

      estimated_jerk = float("nan")
      if (previous_a_ego is not None and previous_a_time is not None and
          a_ego == a_ego):
        dt = now - previous_a_time
        if dt > 0.001:
          estimated_jerk = (a_ego - previous_a_ego) / dt

      if a_ego == a_ego:
        previous_a_ego = a_ego
        previous_a_time = now

      act_accel = as_float(
        get_path(actuators, "accel", float("nan")))
      car_control_active = as_bool(
        get_path(car_control, "active", False))
      controls_active = as_bool(
        get_path(controls, "active", False))

      op_long_unexpected = (
        car_control_active or controls_active or
        (act_accel == act_accel and abs(act_accel) > 0.05)
      )

      row = {
        "t": now - start,
        "wallTime": datetime.now().isoformat(),
        "row": row_count,
        "eventWindow": event_active,
        "eventId": event_id if event_active else "",
        "eventReasons": "|".join(event_reasons) if event_active else "",
        "brakeClassification": brake_classification,

        "rxEvents": rx_events,
        "rxFrames": len(rx_frames),
        "txEvents": tx_events,
        "txFrames": len(tx_frames),
        "rawFramesWritten": raw_written,
        "changeRowsWritten": changes_written,

        "carStateAlive": sm_alive(sm, "carState"),
        "carControlAlive": sm_alive(sm, "carControl"),
        "controlsStateAlive": sm_alive(sm, "controlsState"),
        "radarStateAlive": sm_alive(sm, "radarState"),
        "longitudinalPlanAlive": sm_alive(sm, "longitudinalPlan"),

        "vEgo": v_ego,
        "vEgoKph": v_ego * 3.6,
        "vEgoRaw": as_float(
          get_path(car_state, "vEgoRaw", float("nan"))),
        "aEgo": a_ego,
        "estimatedJerk": estimated_jerk,
        "standstill": standstill,
        "gearShifter": gear,

        "engineRpm": as_float(first_value(
          car_state, ["engineRpm", "engineRPM", "rpm"],
          float("nan"))),
        "engineSpeedRpm": as_float(first_value(
          car_state, ["engineSpeedRpm", "engineSpeed"],
          float("nan"))),
        "motorRpm": as_float(first_value(
          car_state, ["motorRpm", "tractionMotorRpm"],
          float("nan"))),
        "transmissionRpm": as_float(first_value(
          car_state, ["transmissionRpm", "inputShaftRpm"],
          float("nan"))),

        "gas": gas,
        "gasPressed": gas_pressed,
        "gasDelta": gas_delta,
        "brake": as_float(
          get_path(car_state, "brake", float("nan"))),
        "brakePressed": brake_pressed,
        "brakeLights": brake_lights,

        "cruiseAvailable": cruise_available,
        "cruiseEnabled": cruise_enabled,
        "cruiseStandstill": cruise_standstill,
        "cruiseSpeedKph": cruise_speed * 3.6,

        "buttonEventsJson": json.dumps(
          buttons, separators=(",", ":")),
        "buttonEventSignature": buttons_text,

        "carControlEnabled": as_bool(
          get_path(car_control, "enabled", False)),
        "carControlActive": car_control_active,
        "actAccel": act_accel,
        "actGas": as_float(
          get_path(actuators, "gas", float("nan"))),
        "actBrake": as_float(
          get_path(actuators, "brake", float("nan"))),
        "actSpeed": as_float(
          get_path(actuators, "speed", float("nan"))),
        "actSteer": as_float(
          get_path(actuators, "steer", float("nan"))),

        "controlsEnabled": as_bool(
          get_path(controls, "enabled", False)),
        "controlsActive": controls_active,
        "controlsATarget": as_float(
          get_path(controls, "aTarget", float("nan"))),
        "controlsVCruise": as_float(
          get_path(controls, "vCruise", float("nan"))),
        "longControlState": as_text(
          get_path(controls, "longControlState", "")),
        "forceDecel": as_bool(
          get_path(controls, "forceDecel", False)),

        "steeringAngleDeg": as_float(
          get_path(car_state, "steeringAngleDeg", float("nan"))),
        "steeringRateDeg": as_float(
          get_path(car_state, "steeringRateDeg", float("nan"))),
        "steeringTorque": as_float(
          get_path(car_state, "steeringTorque", float("nan"))),
        "steeringTorqueEps": as_float(
          get_path(car_state, "steeringTorqueEps", float("nan"))),
        "steeringPressed": as_bool(
          get_path(car_state, "steeringPressed", False)),
        "steerRatioLive": as_float(
          get_path(live, "steerRatio", float("nan"))),

        "opLongUnexpected": op_long_unexpected,
      }

      row.update(wheel_values(car_state))
      row.update(lead_values(radar))
      row.update(plan_values(plan))
      row.update(rx271)
      row.update(tx271)
      row.update(rx273)
      row.update(tx273)
      row.update(selected_ids("rx", rx_cache))
      row.update(selected_ids("tx", tx_cache))

      summary_writer.writerow(row)
      row_count += 1

      previous_gas = gas
      previous_values = current_values
      previous_strong_accel = strong_accel
      previous_strong_decel = strong_decel

      if row_count % 100 == 0:
        summary_file.flush()
        raw_file.flush()
        change_file.flush()

      if now - last_status >= 5.0:
        print(
          "%.1fs rows=%d speed=%.1f aEgo=%.3f gas=%.3f "
          "cruise=%s brake=%s event=%s raw=%d bits=%d "
          "opLongUnexpected=%s" % (
            row["t"], row_count, row["vEgoKph"], row["aEgo"],
            row["gas"] if row["gas"] == row["gas"] else -1.0,
            "on" if cruise_enabled else "off",
            brake_classification,
            "yes" if event_active else "no",
            raw_written, changes_written,
            "yes" if op_long_unexpected else "no",
          )
        )
        summary_file.flush()
        raw_file.flush()
        change_file.flush()
        last_status = now

      next_sample += 1.0 / RATE_HZ
      delay = next_sample - time.monotonic()
      if delay > 0:
        time.sleep(delay)
      else:
        next_sample = time.monotonic()

  print("")
  print("Logger stopped cleanly.")
  print("Summary rows:", row_count)
  print("Raw CAN frames:", raw_written)
  print("CAN bit changes:", changes_written)
  print("Summary:", summary_path)
  print("Raw CAN:", raw_path)
  print("Bit changes:", change_path)


if __name__ == "__main__":
  main()
