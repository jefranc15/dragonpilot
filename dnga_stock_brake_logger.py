#!/usr/bin/env python3
from __future__ import print_function

import csv
import gzip
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
PRETRIGGER_SECONDS = 5.0
POSTTRIGGER_SECONDS = 8.0
PREBUFFER_MAX_FRAMES = 40000

SERVICES_WANTED = [
  "carState",
  "carControl",
  "controlsState",
  "radarState",
  "longitudinalPlan",
  "liveParameters",
]

SUMMARY_IDS = [
  0x191,
  0x271,
  0x273,
  0x274,
  0x2E4,
  0x2E6,
  0x412,
  0x520,
]

STOP_REQUESTED = False


def request_stop(signum, frame):
  global STOP_REQUESTED
  STOP_REQUESTED = True


def available_services():
  if not service_list:
    return [
      "carState",
      "carControl",
      "controlsState",
      "radarState",
      "liveParameters",
    ]

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


def to_text(value, default=""):
  try:
    return str(value)
  except Exception:
    return default


def sm_message(sm, name):
  try:
    return sm[name]
  except Exception:
    return None


def sm_alive(sm, name):
  try:
    return bool(sm.alive[name])
  except Exception:
    try:
      return bool(sm.alive.get(name, False))
    except Exception:
      return False


def open_socket(name):
  try:
    return messaging.sub_sock(name, conflate=False)
  except TypeError:
    return messaging.sub_sock(name)


def receive_nonblocking(sock):
  try:
    return messaging.recv_one_or_none(sock)
  except Exception:
    return None


def data_bytes(data):
  try:
    return bytes(data)
  except Exception:
    try:
      return bytes(bytearray(ord(x) for x in data))
    except Exception:
      return b""


def hex_data(data):
  return data_bytes(data).hex().upper()


def dnga_checksum(address, data_without_checksum):
  return (address + 8 + 1 + 2 + sum(data_without_checksum)) & 0xFF


def signed_byte(value):
  return value - 256 if value >= 128 else value


def drain_socket(sock, field_name, direction, start_time, cache):
  frames_out = []
  event_count = 0

  while True:
    msg = receive_nonblocking(sock)
    if msg is None:
      break

    event_count += 1

    try:
      frames = getattr(msg, field_name)
    except Exception:
      continue

    mono_now = time.monotonic()
    rel_time = mono_now - start_time
    wall = datetime.now().isoformat()

    for frame in frames:
      try:
        address = int(frame.address)
        bus = int(frame.src)
        raw = data_bytes(frame.dat)
      except Exception:
        continue

      item = {
        "t": rel_time,
        "mono": mono_now,
        "wallTime": wall,
        "direction": direction,
        "address": address,
        "addressHex": "0x%03X" % address,
        "bus": bus,
        "data": raw.hex().upper(),
        "dlc": len(raw),
      }

      frames_out.append(item)
      cache[(address, bus)] = item

  return event_count, frames_out


def latest_for_address(cache, address):
  newest = None
  for (candidate_address, bus), item in cache.items():
    if candidate_address != address:
      continue
    if newest is None or item["mono"] > newest["mono"]:
      newest = item
  return newest


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
    prefix + "271Checksum": "",
    prefix + "271ChecksumExpected": "",
    prefix + "271ChecksumValid": "",
  }

  if item is None:
    return result

  now = time.monotonic()
  result[prefix + "271Raw"] = item["data"]
  result[prefix + "271Bus"] = item["bus"]
  result[prefix + "271AgeMs"] = (now - item["mono"]) * 1000.0

  try:
    data = bytes.fromhex(item["data"])
    if len(data) != 8:
      return result

    state = int(data[1])
    pump_raw = signed_byte(int(data[3]))
    magnitude = (int(data[4]) << 8) | int(data[5])
    counter = (int(data[6]) >> 2) & 0x07
    checksum = int(data[7])
    expected = dnga_checksum(0x271, list(data[:-1]))

    names = {
      0x00: "disabled",
      0x01: "ready_no_brake",
      0x21: "active_brake",
      0x30: "possible_stop_hold_30",
      0x31: "possible_stop_hold_31",
    }

    result.update({
      prefix + "271State": state,
      prefix + "271StateName": names.get(
        state, "unknown_%02X" % state),
      prefix + "271Active": state in (0x21, 0x30, 0x31),
      prefix + "271PumpRaw": pump_raw,
      prefix + "271Pump": pump_raw / 10.0,
      prefix + "271Magnitude": magnitude,
      prefix + "271Counter": counter,
      prefix + "271Checksum": checksum,
      prefix + "271ChecksumExpected": expected,
      prefix + "271ChecksumValid": checksum == expected,
    })
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
    prefix + "273Checksum": "",
    prefix + "273ChecksumExpected": "",
    prefix + "273ChecksumValid": "",
  }

  if item is None:
    return result

  now = time.monotonic()
  result[prefix + "273Raw"] = item["data"]
  result[prefix + "273Bus"] = item["bus"]
  result[prefix + "273AgeMs"] = (now - item["mono"]) * 1000.0

  try:
    data = bytes.fromhex(item["data"])
    if len(data) != 8:
      return result

    checksum = int(data[7])
    expected = dnga_checksum(0x273, list(data[:-1]))

    result.update({
      prefix + "273AccCmdKph":
        ((int(data[2]) << 8) | int(data[3])) * 0.01,
      prefix + "273Byte0": int(data[0]),
      prefix + "273Byte1": int(data[1]),
      prefix + "273Byte4": int(data[4]),
      prefix + "273Byte5": int(data[5]),
      prefix + "273Byte6": int(data[6]),
      prefix + "273Checksum": checksum,
      prefix + "273ChecksumExpected": expected,
      prefix + "273ChecksumValid": checksum == expected,
    })
  except Exception:
    pass

  return result


def selected_can_columns(prefix, cache):
  result = {}
  now = time.monotonic()

  for address in SUMMARY_IDS:
    item = latest_for_address(cache, address)
    key = prefix + "%03X" % address

    if item is None:
      result[key] = ""
      result[key + "Bus"] = ""
      result[key + "AgeMs"] = ""
    else:
      result[key] = item["data"]
      result[key + "Bus"] = item["bus"]
      result[key + "AgeMs"] = (
        now - item["mono"]) * 1000.0

  return result


def extract_lead(radar_state):
  lead = get_path(radar_state, "leadOne", None)
  return {
    "leadStatus": to_bool(get_path(lead, "status", False)),
    "leadDRel": to_float(
      get_path(lead, "dRel", float("nan"))),
    "leadYRel": to_float(
      get_path(lead, "yRel", float("nan"))),
    "leadVRel": to_float(
      get_path(lead, "vRel", float("nan"))),
    "leadVLead": to_float(
      get_path(lead, "vLead", float("nan"))),
    "leadVLeadK": to_float(
      get_path(lead, "vLeadK", float("nan"))),
    "leadALeadK": to_float(
      get_path(lead, "aLeadK", float("nan"))),
    "leadModelProb": to_float(
      get_path(lead, "modelProb", float("nan"))),
    "leadFcw": to_bool(get_path(lead, "fcw", False)),
  }


def plan_values(long_plan):
  result = {
    "planSource": to_text(
      get_path(long_plan, "longitudinalPlanSource", "")),
    "planShouldStop": to_bool(
      get_path(long_plan, "shouldStop", False)),
    "planSpeed0": float("nan"),
    "planSpeedEnd": float("nan"),
    "planAccel0": float("nan"),
    "planAccelEnd": float("nan"),
  }

  speeds = get_path(long_plan, "speeds", [])
  accels = get_path(long_plan, "accels", [])

  try:
    result["planSpeed0"] = float(speeds[0])
    result["planSpeedEnd"] = float(speeds[-1])
  except Exception:
    pass

  try:
    result["planAccel0"] = float(accels[0])
    result["planAccelEnd"] = float(accels[-1])
  except Exception:
    pass

  return result


def wheel_values(car_state):
  wheels = get_path(car_state, "wheelSpeeds", None)
  return {
    "wheelFL": to_float(
      get_path(wheels, "fl", float("nan"))),
    "wheelFR": to_float(
      get_path(wheels, "fr", float("nan"))),
    "wheelRL": to_float(
      get_path(wheels, "rl", float("nan"))),
    "wheelRR": to_float(
      get_path(wheels, "rr", float("nan"))),
  }


def classify_braking(brake_pressed, brake_lights, cruise_enabled,
                     rx271_active, a_ego):
  if brake_pressed:
    return "manual_brake"

  if cruise_enabled and (
      rx271_active or brake_lights or a_ego <= -0.20):
    return "stock_acc_brake"

  if rx271_active:
    return "stock_ecb_active"

  if a_ego <= -0.35:
    return "vehicle_decelerating"

  return "none"


def summary_fields():
  fields = [
    "t",
    "wallTime",
    "row",
    "eventWindow",
    "eventId",
    "eventType",
    "brakeClassification",

    "rxEvents",
    "rxFrames",
    "txEvents",
    "txFrames",
    "rawFramesWritten",

    "carStateAlive",
    "carControlAlive",
    "controlsStateAlive",
    "radarStateAlive",
    "longitudinalPlanAlive",

    "vEgo",
    "vEgoKph",
    "vEgoRaw",
    "aEgo",
    "estimatedJerk",
    "standstill",
    "gearShifter",

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
    "cruiseSpeed",
    "cruiseSpeedKph",

    "carControlEnabled",
    "carControlActive",
    "actAccel",
    "actGas",
    "actBrake",
    "actSpeed",
    "actSteer",

    "controlsEnabled",
    "controlsActive",
    "controlsATarget",
    "controlsVCruise",
    "longControlState",
    "forceDecel",

    "planSource",
    "planShouldStop",
    "planSpeed0",
    "planSpeedEnd",
    "planAccel0",
    "planAccelEnd",

    "leadStatus",
    "leadDRel",
    "leadYRel",
    "leadVRel",
    "leadVLead",
    "leadVLeadK",
    "leadALeadK",
    "leadModelProb",
    "leadFcw",

    "steeringAngleDeg",
    "steeringRateDeg",
    "steeringTorque",
    "steeringTorqueEps",
    "steeringPressed",
    "steerRatioLive",

    "opLongUnexpected",
  ]

  for prefix in ("rx", "tx"):
    fields.extend([
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
      prefix + "271Checksum",
      prefix + "271ChecksumExpected",
      prefix + "271ChecksumValid",

      prefix + "273Raw",
      prefix + "273Bus",
      prefix + "273AgeMs",
      prefix + "273AccCmdKph",
      prefix + "273Byte0",
      prefix + "273Byte1",
      prefix + "273Byte4",
      prefix + "273Byte5",
      prefix + "273Byte6",
      prefix + "273Checksum",
      prefix + "273ChecksumExpected",
      prefix + "273ChecksumValid",
    ])

    for address in SUMMARY_IDS:
      key = prefix + "%03X" % address
      fields.extend([
        key,
        key + "Bus",
        key + "AgeMs",
      ])

  return fields


RAW_FIELDS = [
  "sequence",
  "t",
  "wallTime",
  "eventId",
  "eventType",
  "direction",
  "address",
  "addressHex",
  "bus",
  "dlc",
  "data",
]


def write_raw_frame(writer, frame, event_id, event_type, sequence):
  writer.writerow({
    "sequence": sequence,
    "t": frame["t"],
    "wallTime": frame["wallTime"],
    "eventId": event_id,
    "eventType": event_type,
    "direction": frame["direction"],
    "address": frame["address"],
    "addressHex": frame["addressHex"],
    "bus": frame["bus"],
    "dlc": frame["dlc"],
    "data": frame["data"],
  })


def main():
  signal.signal(signal.SIGINT, request_stop)
  signal.signal(signal.SIGTERM, request_stop)

  os.chdir("/data/openpilot")

  services = available_services()
  sm = messaging.SubMaster(services)

  can_sock = open_socket("can")
  sendcan_sock = open_socket("sendcan")

  timestamp = datetime.now().strftime("%m%d_%H%M%S")
  summary_path = (
    "/data/openpilot/dnga_stock_brake_summary_" +
    timestamp + ".csv.gz"
  )
  raw_path = (
    "/data/openpilot/dnga_stock_brake_raw_" +
    timestamp + ".csv.gz"
  )

  print("DNGA stock/manual brake logger")
  print("Passive only: does not send CAN or modify car files.")
  print("OP toggle should remain OFF for this test.")
  print("Summary:", summary_path)
  print("Raw event CAN:", raw_path)
  print("Raw CAN includes 5 s before and 8 s after brake events.")

  start_time = time.monotonic()
  next_sample = start_time
  last_status = start_time

  rx_cache = {}
  tx_cache = {}
  prebuffer = deque(maxlen=PREBUFFER_MAX_FRAMES)

  row_count = 0
  sequence = 0
  raw_written = 0
  event_id = 0
  event_type = ""
  event_until = 0.0

  previous_a_ego = None
  previous_a_time = None

  with gzip.open(summary_path, "wt", newline="") as summary_file, \
       gzip.open(raw_path, "wt", newline="") as raw_file:

    summary_writer = csv.DictWriter(
      summary_file,
      fieldnames=summary_fields(),
      extrasaction="ignore",
    )
    raw_writer = csv.DictWriter(
      raw_file,
      fieldnames=RAW_FIELDS,
      extrasaction="ignore",
    )

    summary_writer.writeheader()
    raw_writer.writeheader()

    while not STOP_REQUESTED:
      now = time.monotonic()

      try:
        sm.update(0)
      except TypeError:
        sm.update()

      was_event_active = now <= event_until

      rx_events, rx_frames = drain_socket(
        can_sock, "can", "rx", start_time, rx_cache)
      tx_events, tx_frames = drain_socket(
        sendcan_sock, "sendcan", "tx", start_time, tx_cache)

      all_new_frames = rx_frames + tx_frames

      if was_event_active:
        for frame in all_new_frames:
          sequence += 1
          write_raw_frame(
            raw_writer, frame, event_id, event_type, sequence)
          raw_written += 1
      else:
        for frame in all_new_frames:
          prebuffer.append(frame)

      car_state = sm_message(sm, "carState")
      car_control = sm_message(sm, "carControl")
      controls = sm_message(sm, "controlsState")
      radar = sm_message(sm, "radarState")
      long_plan = sm_message(sm, "longitudinalPlan")
      live = sm_message(sm, "liveParameters")

      cruise = get_path(car_state, "cruiseState", None)
      actuators = get_path(car_control, "actuators", None)

      v_ego = to_float(get_path(car_state, "vEgo", 0.0), 0.0)
      a_ego = to_float(
        get_path(car_state, "aEgo", float("nan")))

      brake_pressed = to_bool(
        get_path(car_state, "brakePressed", False))
      brake_lights = to_bool(
        get_path(car_state, "brakeLights", False))
      gas_pressed = to_bool(
        get_path(car_state, "gasPressed", False))
      cruise_enabled = to_bool(
        get_path(cruise, "enabled", False))

      rx271_item = latest_for_address(rx_cache, 0x271)
      tx271_item = latest_for_address(tx_cache, 0x271)
      rx273_item = latest_for_address(rx_cache, 0x273)
      tx273_item = latest_for_address(tx_cache, 0x273)

      rx271 = decode_271(rx271_item, "rx")
      tx271 = decode_271(tx271_item, "tx")
      rx273 = decode_273(rx273_item, "rx")
      tx273 = decode_273(tx273_item, "tx")

      brake_classification = classify_braking(
        brake_pressed,
        brake_lights,
        cruise_enabled,
        to_bool(rx271.get("rx271Active", False)),
        a_ego,
      )

      trigger_event = brake_classification != "none"

      if trigger_event:
        if not was_event_active:
          event_id += 1
          event_type = brake_classification

          for frame in prebuffer:
            sequence += 1
            write_raw_frame(
              raw_writer, frame, event_id, event_type, sequence)
            raw_written += 1
          prebuffer.clear()
        else:
          if brake_classification == "manual_brake":
            event_type = "manual_brake"
          elif event_type != "manual_brake":
            event_type = brake_classification

        event_until = max(
          event_until, now + POSTTRIGGER_SECONDS)

      event_active = now <= event_until

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

      act_accel = to_float(
        get_path(actuators, "accel", float("nan")))
      car_control_active = to_bool(
        get_path(car_control, "active", False))
      controls_active = to_bool(
        get_path(controls, "active", False))

      fresh_tx_long = False
      for item in (tx271_item, tx273_item):
        if item is not None and now - item["mono"] < 0.20:
          fresh_tx_long = True

      op_long_unexpected = (
        car_control_active or
        controls_active or
        (act_accel == act_accel and abs(act_accel) > 0.05)
      )

      row = {
        "t": now - start_time,
        "wallTime": datetime.now().isoformat(),
        "row": row_count,
        "eventWindow": event_active,
        "eventId": event_id if event_active else "",
        "eventType": event_type if event_active else "",
        "brakeClassification": brake_classification,

        "rxEvents": rx_events,
        "rxFrames": len(rx_frames),
        "txEvents": tx_events,
        "txFrames": len(tx_frames),
        "rawFramesWritten": raw_written,

        "carStateAlive": sm_alive(sm, "carState"),
        "carControlAlive": sm_alive(sm, "carControl"),
        "controlsStateAlive": sm_alive(sm, "controlsState"),
        "radarStateAlive": sm_alive(sm, "radarState"),
        "longitudinalPlanAlive": sm_alive(
          sm, "longitudinalPlan"),

        "vEgo": v_ego,
        "vEgoKph": v_ego * 3.6,
        "vEgoRaw": to_float(
          get_path(car_state, "vEgoRaw", float("nan"))),
        "aEgo": a_ego,
        "estimatedJerk": estimated_jerk,
        "standstill": to_bool(
          get_path(car_state, "standstill", False)),
        "gearShifter": to_text(
          get_path(car_state, "gearShifter", "")),

        "gas": to_float(
          get_path(car_state, "gas", float("nan"))),
        "gasPressed": gas_pressed,
        "brake": to_float(
          get_path(car_state, "brake", float("nan"))),
        "brakePressed": brake_pressed,
        "brakeLights": brake_lights,

        "cruiseAvailable": to_bool(
          get_path(cruise, "available", False)),
        "cruiseEnabled": cruise_enabled,
        "cruiseStandstill": to_bool(
          get_path(cruise, "standstill", False)),
        "cruiseSpeed": to_float(
          get_path(cruise, "speed", float("nan"))),
        "cruiseSpeedKph": to_float(
          get_path(cruise, "speed", float("nan"))) * 3.6,

        "carControlEnabled": to_bool(
          get_path(car_control, "enabled", False)),
        "carControlActive": car_control_active,
        "actAccel": act_accel,
        "actGas": to_float(
          get_path(actuators, "gas", float("nan"))),
        "actBrake": to_float(
          get_path(actuators, "brake", float("nan"))),
        "actSpeed": to_float(
          get_path(actuators, "speed", float("nan"))),
        "actSteer": to_float(
          get_path(actuators, "steer", float("nan"))),

        "controlsEnabled": to_bool(
          get_path(controls, "enabled", False)),
        "controlsActive": controls_active,
        "controlsATarget": to_float(
          get_path(controls, "aTarget", float("nan"))),
        "controlsVCruise": to_float(
          get_path(controls, "vCruise", float("nan"))),
        "longControlState": to_text(
          get_path(controls, "longControlState", "")),
        "forceDecel": to_bool(
          get_path(controls, "forceDecel", False)),

        "steeringAngleDeg": to_float(
          get_path(
            car_state, "steeringAngleDeg", float("nan"))),
        "steeringRateDeg": to_float(
          get_path(
            car_state, "steeringRateDeg", float("nan"))),
        "steeringTorque": to_float(
          get_path(
            car_state, "steeringTorque", float("nan"))),
        "steeringTorqueEps": to_float(
          get_path(
            car_state, "steeringTorqueEps", float("nan"))),
        "steeringPressed": to_bool(
          get_path(car_state, "steeringPressed", False)),
        "steerRatioLive": to_float(
          get_path(live, "steerRatio", float("nan"))),

        "opLongUnexpected": op_long_unexpected,
      }

      row.update(wheel_values(car_state))
      row.update(extract_lead(radar))
      row.update(plan_values(long_plan))
      row.update(rx271)
      row.update(tx271)
      row.update(rx273)
      row.update(tx273)
      row.update(selected_can_columns("rx", rx_cache))
      row.update(selected_can_columns("tx", tx_cache))

      summary_writer.writerow(row)
      row_count += 1

      if row_count % 100 == 0:
        summary_file.flush()
        raw_file.flush()

      if now - last_status >= 5.0:
        print(
          "%.1fs rows=%d speed=%.1f aEgo=%.3f "
          "class=%s rx271=%s mag=%s "
          "event=%s raw=%d opLongUnexpected=%s" % (
            row["t"],
            row_count,
            row["vEgoKph"],
            row["aEgo"],
            brake_classification,
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
        next_sample = time.monotonic()

  print("")
  print("Logger stopped cleanly.")
  print("Summary rows:", row_count)
  print("Raw CAN frames saved:", raw_written)
  print("Summary:", summary_path)
  print("Raw event CAN:", raw_path)


if __name__ == "__main__":
  main()
