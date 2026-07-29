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


VERSION = "2.0-light"
RATE_HZ = 20.0
PREBUFFER_SECONDS = 6.0
BRAKE_POST_SECONDS = 6.0
DEFAULT_CAPTURE_SECONDS = 15.0
MAX_PREBUFFER_FRAMES = 70000

BASE = "/data/openpilot"
COMMAND_FILE = BASE + "/dnga_mapping_v2_commands.csv"

SERVICES_WANTED = [
  "carState",
  "carControl",
  "controlsState",
  "radarState",
  "longitudinalPlan",
  "liveParameters",
]

# Decimal CAN addresses. Comments show hexadecimal IDs.
WATCH_IDS = [
  0x0A1,  # brake/wheel context on some DNGA variants
  0x18E,  # GAS_PEDAL, decimal 398 in the available DBC
  0x18F,  # GAS_PEDAL_2, decimal 399
  0x1A0,  # WHEEL_SPEED, decimal 416
  0x1AB,  # BUTTONS, decimal 427
  0x1C0,  # EPS shaft torque, decimal 448
  0x1D0,  # steering command, decimal 464
  0x207,  # PCM_BUTTONS_HYBRID, decimal 519
  0x208,  # PCM_BUTTONS, decimal 520
  0x20C,  # TRANSMISSION, decimal 524
  0x260,  # clean wheel speed, decimal 608
  0x270,  # ADAS_AEB, decimal 624
  0x271,  # ACC_BRAKE, decimal 625
  0x273,  # ACC_CMD_HUD, decimal 627
  0x274,  # LKAS_HUD, decimal 628
  0x520,  # retained for comparison with the earlier logger
]

STOP_REQUESTED = False


def handle_stop(signum, frame):
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


def available_services():
  if not service_list:
    return [
      "carState",
      "carControl",
      "controlsState",
      "radarState",
      "liveParameters",
    ]

  output = []
  for name in SERVICES_WANTED:
    try:
      if name in service_list:
        output.append(name)
    except Exception:
      pass
  return output


def sm_get(sm, name):
  try:
    return sm[name]
  except Exception:
    return None


def sm_alive(sm, name):
  try:
    return bool(sm.alive[name])
  except Exception:
    return False


def open_socket(name):
  try:
    return messaging.sub_sock(name, conflate=False)
  except TypeError:
    return messaging.sub_sock(name)


def recv_one(sock):
  try:
    return messaging.recv_one_or_none(sock)
  except Exception:
    return None


def to_bytes(value):
  try:
    return bytes(value)
  except Exception:
    try:
      return bytes(bytearray(ord(x) for x in value))
    except Exception:
      return b""


def bit(data, byte_index, bit_index):
  try:
    return (data[byte_index] >> bit_index) & 1
  except Exception:
    return 0


def signed_byte(value):
  return value - 256 if value >= 128 else value


def dnga_checksum(address, data_without_checksum):
  return (address + 8 + 1 + 2 + sum(data_without_checksum)) & 0xFF


def message_log_ns(msg, fallback_ns):
  try:
    value = int(msg.logMonoTime)
    return value if value > 0 else fallback_ns
  except Exception:
    return fallback_ns


def drain_socket(sock, field_name, direction, start_mono_ns, cache):
  event_count = 0
  frames_out = []

  while True:
    msg = recv_one(sock)
    if msg is None:
      break

    event_count += 1
    recv_ns = time.monotonic_ns()
    log_ns = message_log_ns(msg, recv_ns)

    try:
      frames = getattr(msg, field_name)
    except Exception:
      continue

    wall = datetime.now().isoformat()
    t_original = (log_ns - start_mono_ns) / 1e9
    t_received = (recv_ns - start_mono_ns) / 1e9

    for frame in frames:
      try:
        address = int(frame.address)
        bus = int(frame.src)
        raw = to_bytes(frame.dat)
      except Exception:
        continue

      item = {
        "t": t_original,
        "tReceived": t_received,
        "logMonoTimeNs": log_ns,
        "wallTime": wall,
        "direction": direction,
        "address": address,
        "addressHex": "0x%03X" % address,
        "bus": bus,
        "dlc": len(raw),
        "data": raw.hex().upper(),
        "raw": raw,
        "recvMonoNs": recv_ns,
      }

      frames_out.append(item)
      cache[(address, bus)] = item

  return event_count, frames_out


def latest_address(cache, address):
  newest = None
  for (candidate, bus), item in cache.items():
    if candidate != address:
      continue
    if newest is None or item["logMonoTimeNs"] > newest["logMonoTimeNs"]:
      newest = item
  return newest


def raw_and_bus(item, prefix):
  if item is None:
    return {
      prefix + "Raw": "",
      prefix + "Bus": "",
      prefix + "AgeMs": "",
    }

  return {
    prefix + "Raw": item["data"],
    prefix + "Bus": item["bus"],
    prefix + "AgeMs":
      (time.monotonic_ns() - item["logMonoTimeNs"]) / 1e6,
  }


def decode_207(item, prefix):
  output = raw_and_bus(item, prefix + "207")
  output.update({
    prefix + "207GasPressedBit": "",
    prefix + "207ResPlus": "",
    prefix + "207SetMinus": "",
    prefix + "207Cancel": "",
  })

  if item is None:
    return output

  data = item["raw"]
  output.update({
    prefix + "207GasPressedBit": bit(data, 1, 4),
    prefix + "207ResPlus": bit(data, 0, 6),
    prefix + "207SetMinus": bit(data, 0, 5),
    prefix + "207Cancel": bit(data, 0, 4),
  })
  return output


def decode_208(item, prefix):
  output = raw_and_bus(item, prefix + "208")
  output.update({
    prefix + "208AccRdy": "",
    prefix + "208ResPlus": "",
    prefix + "208SetMinus": "",
    prefix + "208Cancel": "",
    prefix + "208PedalDepressed": "",
    prefix + "208NewSignal1": "",
    prefix + "208NewSignal2": "",
    prefix + "208Counter": "",
  })

  if item is None:
    return output

  data = item["raw"]
  output.update({
    prefix + "208AccRdy": bit(data, 1, 1),
    prefix + "208ResPlus": bit(data, 0, 6),
    prefix + "208SetMinus": bit(data, 0, 5),
    prefix + "208Cancel": bit(data, 0, 4),
    prefix + "208PedalDepressed": bit(data, 1, 4),
    prefix + "208NewSignal1": bit(data, 1, 3),
    prefix + "208NewSignal2": bit(data, 3, 7),
    prefix + "208Counter":
      (data[3] & 0x0F) if len(data) > 3 else "",
  })
  return output


def decode_1ab(item, prefix):
  output = raw_and_bus(item, prefix + "1AB")
  output.update({
    prefix + "1ABPowerButton": "",
    prefix + "1ABLkcButton": "",
    prefix + "1ABDistanceButton": "",
  })

  if item is None:
    return output

  data = item["raw"]
  output.update({
    prefix + "1ABPowerButton": bit(data, 3, 4),
    prefix + "1ABLkcButton": bit(data, 4, 3),
    prefix + "1ABDistanceButton": bit(data, 0, 7),
  })
  return output


def decode_271(item, prefix):
  output = raw_and_bus(item, prefix + "271")
  output.update({
    prefix + "271State": "",
    prefix + "271StateName": "",
    prefix + "271Active": False,
    prefix + "271PumpRaw": "",
    prefix + "271Pump": "",
    prefix + "271Magnitude": "",
    prefix + "271Counter": "",
    prefix + "271ChecksumValid": "",
  })

  if item is None:
    return output

  try:
    data = item["raw"]
    if len(data) != 8:
      return output

    state = int(data[1])
    names = {
      0x00: "disabled",
      0x01: "ready_no_brake",
      0x20: "possible_brake_20",
      0x21: "active_brake",
      0x30: "possible_hold_30",
      0x31: "possible_hold_31",
    }

    output.update({
      prefix + "271State": state,
      prefix + "271StateName":
        names.get(state, "unknown_%02X" % state),
      prefix + "271Active": state in (0x20, 0x21, 0x30, 0x31),
      prefix + "271PumpRaw": signed_byte(int(data[3])),
      prefix + "271Pump": signed_byte(int(data[3])) / 10.0,
      prefix + "271Magnitude":
        (int(data[4]) << 8) | int(data[5]),
      prefix + "271Counter":
        (int(data[6]) >> 2) & 0x07,
      prefix + "271ChecksumValid":
        int(data[7]) == dnga_checksum(0x271, list(data[:-1])),
    })
  except Exception:
    pass

  return output


def decode_273(item, prefix):
  output = raw_and_bus(item, prefix + "273")
  output.update({
    prefix + "273SetSpeedKph": "",
    prefix + "273FollowDistance": "",
    prefix + "273Lead": "",
    prefix + "273AccCmdKph": "",
    prefix + "273IsAccel": "",
    prefix + "273IsDecel": "",
    prefix + "273SetMe12": "",
    prefix + "273Set1WhenEngage": "",
    prefix + "273Set0WhenEngage": "",
    prefix + "273SetMe1": "",
    prefix + "273Unknown1": "",
    prefix + "273Unknown2": "",
    prefix + "273ChecksumValid": "",
  })

  if item is None:
    return output

  try:
    data = item["raw"]
    if len(data) != 8:
      return output

    output.update({
      prefix + "273SetSpeedKph": int(data[0]),
      prefix + "273FollowDistance": int(data[1]) & 0x03,
      prefix + "273Lead": bit(data, 1, 3),
      prefix + "273AccCmdKph":
        ((int(data[2]) << 8) | int(data[3])) * 0.01,
      prefix + "273IsAccel": bit(data, 4, 6),
      prefix + "273IsDecel": bit(data, 4, 5),
      prefix + "273SetMe12": bit(data, 1, 1),
      prefix + "273Set1WhenEngage": bit(data, 1, 5),
      prefix + "273Set0WhenEngage": bit(data, 5, 0),
      prefix + "273SetMe1": bit(data, 6, 6),
      prefix + "273Unknown1": bit(data, 5, 7),
      prefix + "273Unknown2": bit(data, 4, 1),
      prefix + "273ChecksumValid":
        int(data[7]) == dnga_checksum(0x273, list(data[:-1])),
    })
  except Exception:
    pass

  return output


def decode_274(item, prefix):
  output = raw_and_bus(item, prefix + "274")
  output.update({
    prefix + "274LkasSet": "",
    prefix + "274LkasEngaged": "",
    prefix + "274HoldWarning": "",
    prefix + "274LdaOff": "",
    prefix + "274LdaAlert": "",
    prefix + "274AebAlarm": "",
    prefix + "274AebBrake": "",
    prefix + "274FrontDepart": "",
  })

  if item is None:
    return output

  data = item["raw"]
  output.update({
    prefix + "274LkasSet": bit(data, 2, 3),
    prefix + "274LkasEngaged": bit(data, 3, 6),
    prefix + "274HoldWarning": bit(data, 2, 2),
    prefix + "274LdaOff": bit(data, 1, 5),
    prefix + "274LdaAlert": bit(data, 1, 6),
    prefix + "274AebAlarm": bit(data, 1, 0),
    prefix + "274AebBrake": bit(data, 2, 7),
    prefix + "274FrontDepart": bit(data, 1, 1),
  })
  return output


def button_events(car_state):
  output = []
  try:
    for event in get_path(car_state, "buttonEvents", []):
      try:
        event_type = event.type.raw
      except Exception:
        event_type = as_text(get_path(event, "type", ""))
      output.append({
        "type": event_type,
        "pressed": as_bool(get_path(event, "pressed", False)),
      })
  except Exception:
    pass
  return output


def lead_values(radar):
  lead = get_path(radar, "leadOne", None)
  return {
    "leadStatus": as_bool(get_path(lead, "status", False)),
    "leadDRel": as_float(get_path(lead, "dRel", float("nan"))),
    "leadVRel": as_float(get_path(lead, "vRel", float("nan"))),
    "leadVLead": as_float(get_path(lead, "vLead", float("nan"))),
    "leadALeadK": as_float(get_path(lead, "aLeadK", float("nan"))),
    "leadModelProb":
      as_float(get_path(lead, "modelProb", float("nan"))),
  }


def plan_values(plan):
  output = {
    "planSource":
      as_text(get_path(plan, "longitudinalPlanSource", "")),
    "planShouldStop":
      as_bool(get_path(plan, "shouldStop", False)),
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
        if len(parts) < 3 or parts[0] != "capture":
          continue

        try:
          duration = max(1.0, min(float(parts[1]), 120.0))
        except Exception:
          duration = DEFAULT_CAPTURE_SECONDS

        commands.append((duration, parts[2].strip()))
      offset = stream.tell()
  except Exception:
    pass
  return offset, commands


def add_reason(reasons, reason):
  if reason and reason not in reasons:
    reasons.append(reason)


RAW_FIELDS = [
  "sequence",
  "t",
  "tReceived",
  "logMonoTimeNs",
  "wallTime",
  "eventId",
  "eventReasons",
  "direction",
  "address",
  "addressHex",
  "bus",
  "dlc",
  "data",
]


SUMMARY_FIELDS = [
  "t",
  "wallTime",
  "row",
  "loggerVersion",
  "eventWindow",
  "eventId",
  "eventReasons",
  "rawFramesWritten",
  "rxFramesThisCycle",
  "txFramesThisCycle",

  "vEgo",
  "vEgoKph",
  "vEgoRaw",
  "aEgo",
  "estimatedJerk",
  "standstill",
  "gearShifter",

  "engineRpm",
  "engineSpeedRpm",
  "motorRpm",
  "transmissionRpm",

  "gas",
  "gasPressed",
  "brake",
  "brakePressed",
  "brakeLights",

  "wheelFL",
  "wheelFR",
  "wheelRL",
  "wheelRR",

  "cruiseAvailable",
  "cruiseEnabled",
  "cruiseStandstill",
  "cruiseSpeedKph",

  "buttonEventsJson",

  "carControlEnabled",
  "carControlActive",
  "controlsEnabled",
  "controlsActive",
  "actAccel",
  "actGas",
  "actBrake",
  "actSpeed",
  "controlsATarget",
  "longControlState",

  "leadStatus",
  "leadDRel",
  "leadVRel",
  "leadVLead",
  "leadALeadK",
  "leadModelProb",

  "planSource",
  "planShouldStop",
  "planSpeed0",
  "planSpeedEnd",
  "planAccel0",
  "planAccelEnd",

  "steeringAngleDeg",
  "steeringRateDeg",
  "steeringTorque",
  "steeringTorqueEps",
  "steeringPressed",
  "steerRatioLive",

  "opLongUnexpected",
]

for prefix in ("rx", "tx"):
  SUMMARY_FIELDS.extend([
    prefix + "207Raw",
    prefix + "207Bus",
    prefix + "207AgeMs",
    prefix + "207GasPressedBit",
    prefix + "207ResPlus",
    prefix + "207SetMinus",
    prefix + "207Cancel",

    prefix + "208Raw",
    prefix + "208Bus",
    prefix + "208AgeMs",
    prefix + "208AccRdy",
    prefix + "208ResPlus",
    prefix + "208SetMinus",
    prefix + "208Cancel",
    prefix + "208PedalDepressed",
    prefix + "208NewSignal1",
    prefix + "208NewSignal2",
    prefix + "208Counter",

    prefix + "1ABRaw",
    prefix + "1ABBus",
    prefix + "1ABAgeMs",
    prefix + "1ABPowerButton",
    prefix + "1ABLkcButton",
    prefix + "1ABDistanceButton",

    prefix + "271Raw",
    prefix + "271Bus",
    prefix + "271AgeMs",
    prefix + "271State",
    prefix + "271StateName",
    prefix + "271Active",
    prefix + "271PumpRaw",
    prefix + "271Pump",
    prefix + "271Magnitude",
    prefix + "271Counter",
    prefix + "271ChecksumValid",

    prefix + "273Raw",
    prefix + "273Bus",
    prefix + "273AgeMs",
    prefix + "273SetSpeedKph",
    prefix + "273FollowDistance",
    prefix + "273Lead",
    prefix + "273AccCmdKph",
    prefix + "273IsAccel",
    prefix + "273IsDecel",
    prefix + "273SetMe12",
    prefix + "273Set1WhenEngage",
    prefix + "273Set0WhenEngage",
    prefix + "273SetMe1",
    prefix + "273Unknown1",
    prefix + "273Unknown2",
    prefix + "273ChecksumValid",

    prefix + "274Raw",
    prefix + "274Bus",
    prefix + "274AgeMs",
    prefix + "274LkasSet",
    prefix + "274LkasEngaged",
    prefix + "274HoldWarning",
    prefix + "274LdaOff",
    prefix + "274LdaAlert",
    prefix + "274AebAlarm",
    prefix + "274AebBrake",
    prefix + "274FrontDepart",
  ])

  for address in WATCH_IDS:
    key = prefix + "%03XRaw" % address
    SUMMARY_FIELDS.extend([key, key[:-3] + "Bus", key[:-3] + "AgeMs"])


EVENT_FIELDS = [
  "eventId",
  "eventStartT",
  "eventEndT",
  "eventReasons",
  "rawFramesAtStart",
  "rawFramesAtEnd",
]


def watch_columns(prefix, cache):
  output = {}
  now_ns = time.monotonic_ns()

  for address in WATCH_IDS:
    item = latest_address(cache, address)
    stem = prefix + "%03X" % address

    if item is None:
      output[stem + "Raw"] = ""
      output[stem + "Bus"] = ""
      output[stem + "AgeMs"] = ""
    else:
      output[stem + "Raw"] = item["data"]
      output[stem + "Bus"] = item["bus"]
      output[stem + "AgeMs"] = (
        now_ns - item["logMonoTimeNs"]) / 1e6

  return output


def write_raw(writer, frame, event_id, reasons, sequence):
  writer.writerow({
    "sequence": sequence,
    "t": frame["t"],
    "tReceived": frame["tReceived"],
    "logMonoTimeNs": frame["logMonoTimeNs"],
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


def main():
  signal.signal(signal.SIGINT, handle_stop)
  signal.signal(signal.SIGTERM, handle_stop)

  os.chdir(BASE)

  services = available_services()
  sm = messaging.SubMaster(services)
  can_sock = open_socket("can")
  sendcan_sock = open_socket("sendcan")

  stamp = datetime.now().strftime("%m%d_%H%M%S")
  summary_path = (
    BASE + "/dnga_mapping_v2_summary_" + stamp + ".csv.gz")
  raw_path = (
    BASE + "/dnga_mapping_v2_raw_" + stamp + ".csv.gz")
  events_path = (
    BASE + "/dnga_mapping_v2_events_" + stamp + ".csv")

  print("DNGA mapping logger %s" % VERSION)
  print("Passive/read-only. It does not transmit CAN.")
  print("No vehicle files are modified.")
  print("Summary:", summary_path)
  print("Raw event CAN:", raw_path)
  print("Events:", events_path)
  print("Automatic raw capture: manual brake and stock 0x271 brake.")
  print("Buttons/HUD: use the ctl capture command.")

  start_ns = time.monotonic_ns()
  start_seconds = start_ns / 1e9
  next_sample = time.monotonic()
  last_status = time.monotonic()

  rx_cache = {}
  tx_cache = {}
  prebuffer = deque(maxlen=MAX_PREBUFFER_FRAMES)

  row_count = 0
  raw_sequence = 0
  raw_written = 0

  event_id = 0
  event_reasons = []
  event_until = 0.0
  event_start_t = None
  event_start_raw = 0
  event_logged = False

  command_offset = 0
  previous_brake_pressed = False
  previous_rx271_active = False
  previous_a_ego = None
  previous_a_time = None

  with gzip.open(summary_path, "wt", newline="") as summary_file, \
       gzip.open(raw_path, "wt", newline="") as raw_file, \
       open(events_path, "w", newline="") as events_file:

    summary_writer = csv.DictWriter(
      summary_file,
      fieldnames=SUMMARY_FIELDS,
      extrasaction="ignore",
    )
    raw_writer = csv.DictWriter(
      raw_file,
      fieldnames=RAW_FIELDS,
      extrasaction="ignore",
    )
    events_writer = csv.DictWriter(
      events_file,
      fieldnames=EVENT_FIELDS,
      extrasaction="ignore",
    )

    summary_writer.writeheader()
    raw_writer.writeheader()
    events_writer.writeheader()

    while not STOP_REQUESTED:
      now = time.monotonic()
      event_active_before = now <= event_until

      try:
        sm.update(0)
      except TypeError:
        sm.update()

      rx_events, rx_frames = drain_socket(
        can_sock, "can", "rx", start_ns, rx_cache)
      tx_events, tx_frames = drain_socket(
        sendcan_sock, "sendcan", "tx", start_ns, tx_cache)
      new_frames = rx_frames + tx_frames

      car_state = sm_get(sm, "carState")
      car_control = sm_get(sm, "carControl")
      controls = sm_get(sm, "controlsState")
      radar = sm_get(sm, "radarState")
      plan = sm_get(sm, "longitudinalPlan")
      live = sm_get(sm, "liveParameters")

      cruise = get_path(car_state, "cruiseState", None)
      actuators = get_path(car_control, "actuators", None)

      rx271_item = latest_address(rx_cache, 0x271)
      tx271_item = latest_address(tx_cache, 0x271)
      rx271 = decode_271(rx271_item, "rx")
      tx271 = decode_271(tx271_item, "tx")

      brake_pressed = as_bool(
        get_path(car_state, "brakePressed", False))
      rx271_active = as_bool(rx271.get("rx271Active", False))

      reasons = []
      requested_post = BRAKE_POST_SECONDS

      if brake_pressed:
        add_reason(reasons, "manual_brake")

      if rx271_active:
        add_reason(
          reasons,
          "stock_271:" + as_text(
            rx271.get("rx271StateName", "active")),
        )

      # Rising edges get explicit labels even if the state remains active.
      if brake_pressed and not previous_brake_pressed:
        add_reason(reasons, "manual_brake_start")

      if rx271_active and not previous_rx271_active:
        add_reason(reasons, "stock_271_start")

      command_offset, commands = poll_commands(command_offset)
      for duration, label in commands:
        add_reason(reasons, "capture:" + label)
        requested_post = max(requested_post, duration)

      if reasons:
        if not event_active_before:
          event_id += 1
          event_reasons = []
          event_start_t = now - start_seconds
          event_start_raw = raw_written
          event_logged = False

          for reason in reasons:
            add_reason(event_reasons, reason)

          reason_text = "|".join(event_reasons)

          for frame in prebuffer:
            raw_sequence += 1
            write_raw(
              raw_writer,
              frame,
              event_id,
              reason_text,
              raw_sequence,
            )
            raw_written += 1
          prebuffer.clear()
        else:
          for reason in reasons:
            add_reason(event_reasons, reason)

        event_until = max(event_until, now + requested_post)

      event_active = now <= event_until
      reason_text = "|".join(event_reasons)

      if event_active:
        for frame in new_frames:
          raw_sequence += 1
          write_raw(
            raw_writer,
            frame,
            event_id,
            reason_text,
            raw_sequence,
          )
          raw_written += 1
      else:
        for frame in new_frames:
          prebuffer.append(frame)

      if (
          not event_active and
          event_id > 0 and
          event_start_t is not None and
          not event_logged
      ):
        events_writer.writerow({
          "eventId": event_id,
          "eventStartT": event_start_t,
          "eventEndT": now - start_seconds,
          "eventReasons": reason_text,
          "rawFramesAtStart": event_start_raw,
          "rawFramesAtEnd": raw_written,
        })
        events_file.flush()
        event_logged = True

      v_ego = as_float(get_path(car_state, "vEgo", 0.0), 0.0)
      a_ego = as_float(
        get_path(car_state, "aEgo", float("nan")))

      estimated_jerk = float("nan")
      if (
          previous_a_ego is not None and
          previous_a_time is not None and
          a_ego == a_ego
      ):
        dt = now - previous_a_time
        if dt > 0.001:
          estimated_jerk = (a_ego - previous_a_ego) / dt

      if a_ego == a_ego:
        previous_a_ego = a_ego
        previous_a_time = now

      cruise_speed = as_float(
        get_path(cruise, "speed", float("nan")))
      car_control_active = as_bool(
        get_path(car_control, "active", False))
      controls_active = as_bool(
        get_path(controls, "active", False))
      act_accel = as_float(
        get_path(actuators, "accel", float("nan")))

      op_long_unexpected = (
        car_control_active or
        controls_active or
        (act_accel == act_accel and abs(act_accel) > 0.05)
      )

      row = {
        "t": now - start_seconds,
        "wallTime": datetime.now().isoformat(),
        "row": row_count,
        "loggerVersion": VERSION,
        "eventWindow": event_active,
        "eventId": event_id if event_active else "",
        "eventReasons": reason_text if event_active else "",
        "rawFramesWritten": raw_written,
        "rxFramesThisCycle": len(rx_frames),
        "txFramesThisCycle": len(tx_frames),

        "vEgo": v_ego,
        "vEgoKph": v_ego * 3.6,
        "vEgoRaw": as_float(
          get_path(car_state, "vEgoRaw", float("nan"))),
        "aEgo": a_ego,
        "estimatedJerk": estimated_jerk,
        "standstill":
          as_bool(get_path(car_state, "standstill", False)),
        "gearShifter":
          as_text(get_path(car_state, "gearShifter", "")),

        "engineRpm": as_float(first_value(
          car_state,
          ["engineRpm", "engineRPM", "rpm"],
          float("nan"),
        )),
        "engineSpeedRpm": as_float(first_value(
          car_state,
          ["engineSpeedRpm", "engineSpeed"],
          float("nan"),
        )),
        "motorRpm": as_float(first_value(
          car_state,
          ["motorRpm", "tractionMotorRpm"],
          float("nan"),
        )),
        "transmissionRpm": as_float(first_value(
          car_state,
          ["transmissionRpm", "inputShaftRpm"],
          float("nan"),
        )),

        "gas": as_float(
          get_path(car_state, "gas", float("nan"))),
        "gasPressed":
          as_bool(get_path(car_state, "gasPressed", False)),
        "brake": as_float(
          get_path(car_state, "brake", float("nan"))),
        "brakePressed": brake_pressed,
        "brakeLights":
          as_bool(get_path(car_state, "brakeLights", False)),

        "cruiseAvailable":
          as_bool(get_path(cruise, "available", False)),
        "cruiseEnabled":
          as_bool(get_path(cruise, "enabled", False)),
        "cruiseStandstill":
          as_bool(get_path(cruise, "standstill", False)),
        "cruiseSpeedKph": cruise_speed * 3.6,

        "buttonEventsJson": json.dumps(
          button_events(car_state),
          separators=(",", ":"),
        ),

        "carControlEnabled":
          as_bool(get_path(car_control, "enabled", False)),
        "carControlActive": car_control_active,
        "controlsEnabled":
          as_bool(get_path(controls, "enabled", False)),
        "controlsActive": controls_active,
        "actAccel": act_accel,
        "actGas": as_float(
          get_path(actuators, "gas", float("nan"))),
        "actBrake": as_float(
          get_path(actuators, "brake", float("nan"))),
        "actSpeed": as_float(
          get_path(actuators, "speed", float("nan"))),
        "controlsATarget": as_float(
          get_path(controls, "aTarget", float("nan"))),
        "longControlState": as_text(
          get_path(controls, "longControlState", "")),

        "steeringAngleDeg": as_float(
          get_path(
            car_state, "steeringAngleDeg", float("nan"))),
        "steeringRateDeg": as_float(
          get_path(
            car_state, "steeringRateDeg", float("nan"))),
        "steeringTorque": as_float(
          get_path(
            car_state, "steeringTorque", float("nan"))),
        "steeringTorqueEps": as_float(
          get_path(
            car_state, "steeringTorqueEps", float("nan"))),
        "steeringPressed": as_bool(
          get_path(car_state, "steeringPressed", False)),
        "steerRatioLive": as_float(
          get_path(live, "steerRatio", float("nan"))),

        "opLongUnexpected": op_long_unexpected,
      }

      row.update(wheel_values(car_state))
      row.update(lead_values(radar))
      row.update(plan_values(plan))

      for prefix, cache in (("rx", rx_cache), ("tx", tx_cache)):
        row.update(decode_207(
          latest_address(cache, 0x207), prefix))
        row.update(decode_208(
          latest_address(cache, 0x208), prefix))
        row.update(decode_1ab(
          latest_address(cache, 0x1AB), prefix))
        row.update(decode_271(
          latest_address(cache, 0x271), prefix))
        row.update(decode_273(
          latest_address(cache, 0x273), prefix))
        row.update(decode_274(
          latest_address(cache, 0x274), prefix))
        row.update(watch_columns(prefix, cache))

      summary_writer.writerow(row)
      row_count += 1

      previous_brake_pressed = brake_pressed
      previous_rx271_active = rx271_active

      if row_count % 100 == 0:
        summary_file.flush()
        raw_file.flush()

      if now - last_status >= 5.0:
        print(
          "%.1fs rows=%d speed=%.1f aEgo=%.3f "
          "rx208_ACC_RDY=%s PWR=%s rx271=%s mag=%s "
          "event=%s raw=%d opLongUnexpected=%s" % (
            row["t"],
            row_count,
            row["vEgoKph"],
            row["aEgo"],
            row.get("rx208AccRdy", ""),
            row.get("rx1ABPowerButton", ""),
            row.get("rx271StateName", ""),
            row.get("rx271Magnitude", ""),
            "yes" if event_active else "no",
            raw_written,
            "yes" if op_long_unexpected else "no",
          )
        )
        summary_file.flush()
        raw_file.flush()
        last_status = now

      next_sample += 1.0 / RATE_HZ
      delay = next_sample - time.monotonic()
      if delay > 0:
        time.sleep(delay)
      else:
        # Do not attempt to replay missed summary cycles.
        next_sample = time.monotonic()

  now = time.monotonic()

  if (
      event_id > 0 and
      event_start_t is not None and
      not event_logged
  ):
    events_writer.writerow({
      "eventId": event_id,
      "eventStartT": event_start_t,
      "eventEndT": now - start_seconds,
      "eventReasons": "|".join(event_reasons),
      "rawFramesAtStart": event_start_raw,
      "rawFramesAtEnd": raw_written,
    })

  print("")
  print("Logger stopped cleanly.")
  print("Summary rows:", row_count)
  print("Raw event frames:", raw_written)
  print("Summary:", summary_path)
  print("Raw CAN:", raw_path)
  print("Events:", events_path)


if __name__ == "__main__":
  main()
