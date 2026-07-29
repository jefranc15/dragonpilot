#!/usr/bin/env python3
from __future__ import print_function

import csv
import gzip
import os
import signal
import time
from datetime import datetime

import cereal.messaging as messaging
from common.realtime import Ratekeeper

try:
  from cereal.services import service_list
except Exception:
  service_list = {}

RATE_HZ = 20
OUTPUT_DIR = "/data/openpilot"
CAN_IDS = (0x271, 0x273, 0x274, 0x2E4)
STOP_REQUESTED = False

# Must match the v2.5e controller currently being tested.
BRAKE_ON_THRESHOLD = 0.25
BRAKE_OFF_THRESHOLD = 0.08
BRAKE_ON_COUNT = 3
BRAKE_OFF_COUNT = 2
ENGAGE_BLOCK_SECONDS = 2.0
CUT_IN_SECONDS = 0.5


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
  optional = ["radarState", "longitudinalPlan", "liveParameters"]

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


def dnga_checksum(address, data_without_checksum):
  # Same checksum used by dngacan.py.
  return (address + 8 + 1 + 2 + sum(data_without_checksum)) & 0xFF


def signed_byte(value):
  return value - 256 if value >= 128 else value


def decode_271(hex_string):
  result = {
    "tx271State": "",
    "tx271StateName": "",
    "tx271Active": False,
    "tx271PumpRaw": "",
    "tx271Pump": "",
    "tx271Magnitude": "",
    "tx271Counter": "",
    "tx271Checksum": "",
    "tx271ChecksumExpected": "",
    "tx271ChecksumValid": "",
  }

  try:
    data = bytes.fromhex(hex_string)
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
      "tx271State": state,
      "tx271StateName": names.get(state, "unknown_%02X" % state),
      "tx271Active": state == 0x21,
      "tx271PumpRaw": pump_raw,
      "tx271Pump": pump_raw / 10.0,
      "tx271Magnitude": magnitude,
      "tx271Counter": counter,
      "tx271Checksum": checksum,
      "tx271ChecksumExpected": expected,
      "tx271ChecksumValid": checksum == expected,
    })
  except Exception:
    pass

  return result


def decode_checksum(address, hex_string):
  try:
    data = bytes.fromhex(hex_string)
    if len(data) != 8:
      return ""
    return int(data[-1]) == dnga_checksum(address, list(data[:-1]))
  except Exception:
    return ""


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


def lateral_values(controls):
  result = {
    "latType": "",
    "latActive": False,
    "latOutput": float("nan"),
    "latSaturated": False,
    "desiredAngleDeg": float("nan"),
    "controllerAngleDeg": float("nan"),
    "angleErrorDeg": float("nan"),
    "latP": float("nan"),
    "latI": float("nan"),
    "latF": float("nan"),
  }

  state_union = get_path(controls, "lateralControlState", None)
  if state_union is None:
    return result

  try:
    state_name = state_union.which()
    state = getattr(state_union, state_name)
  except Exception:
    return result

  result["latType"] = state_name
  result["latActive"] = to_bool(get_path(state, "active", False))
  result["latOutput"] = to_float(get_path(state, "output", float("nan")))
  result["latSaturated"] = to_bool(get_path(state, "saturated", False))
  result["desiredAngleDeg"] = to_float(
    get_path(state, "steeringAngleDesiredDeg", float("nan")))
  result["controllerAngleDeg"] = to_float(
    get_path(state, "steeringAngleDeg", float("nan")))
  result["angleErrorDeg"] = to_float(get_path(state, "angleError", float("nan")))
  result["latP"] = to_float(get_path(state, "p", float("nan")))
  result["latI"] = to_float(get_path(state, "i", float("nan")))
  result["latF"] = to_float(get_path(state, "f", float("nan")))
  return result


class V25EInference(object):
  """Reconstructs v2.5e brake gating without modifying carcontroller.py."""

  def __init__(self):
    self.initialized = False
    self.prev_enabled = False
    self.prev_lead = False
    self.engage_time = None
    self.cutin_until = 0.0
    self.req_counter = 0
    self.release_counter = 0
    self.brake_active = False

  def update(self, now, enabled, cruise_enabled, standstill, v_ego,
             gas_pressed, brake_pressed, hud_lead_visible, act_accel, a_ego):
    if not self.initialized:
      self.initialized = True
      self.prev_enabled = enabled
      self.prev_lead = hud_lead_visible
      # If logging starts while already engaged, do not invent a new 2 s block.
      self.engage_time = None

    enabled_rise = enabled and not self.prev_enabled
    if enabled_rise:
      self.engage_time = now

    lead_rise = hud_lead_visible and not self.prev_lead
    if lead_rise and v_ego > 7.0:
      self.cutin_until = now + CUT_IN_SECONDS

    block_remaining = 0.0
    if self.engage_time is not None:
      block_remaining = max(0.0, ENGAGE_BLOCK_SECONDS - (now - self.engage_time))

    cutin_remaining = max(0.0, self.cutin_until - now)
    apply_brake = max(0.0, -act_accel)
    cutin_ok = cutin_remaining <= 0.0 or (apply_brake >= 0.65 and a_ego < -0.10)

    basic_allowed = (
      enabled and
      cruise_enabled and
      not standstill and
      v_ego > 1.0 and
      act_accel < 0.0 and
      not gas_pressed and
      not brake_pressed
    )

    brake_allowed = basic_allowed and block_remaining <= 0.0 and cutin_ok

    if brake_allowed:
      if apply_brake > BRAKE_ON_THRESHOLD:
        self.req_counter = min(self.req_counter + 1, 10)
        self.release_counter = 0
      elif apply_brake < BRAKE_OFF_THRESHOLD:
        self.release_counter = min(self.release_counter + 1, 10)
        self.req_counter = 0
      # Between thresholds, mirror controller hysteresis and retain state/counters.
    else:
      self.req_counter = 0
      self.release_counter = 0
      self.brake_active = False

    if not self.brake_active and self.req_counter >= BRAKE_ON_COUNT:
      self.brake_active = True
    elif self.brake_active and self.release_counter >= BRAKE_OFF_COUNT:
      self.brake_active = False

    reason = "allowed_or_below_threshold"
    if not enabled:
      reason = "op_not_enabled"
    elif not cruise_enabled:
      reason = "cruise_not_enabled"
    elif standstill:
      reason = "standstill"
    elif v_ego <= 1.0:
      reason = "speed_below_1ms"
    elif gas_pressed:
      reason = "driver_gas_pressed"
    elif brake_pressed:
      reason = "driver_brake_pressed"
    elif act_accel >= 0.0:
      reason = "op_accel_nonnegative"
    elif block_remaining > 0.0:
      reason = "post_engagement_2s_block"
    elif not cutin_ok:
      reason = "new_lead_0p5s_cutin_block"
    elif apply_brake <= BRAKE_ON_THRESHOLD and not self.brake_active:
      reason = "negative_request_below_0p25_threshold"
    elif self.req_counter < BRAKE_ON_COUNT and not self.brake_active:
      reason = "three_cycle_activation_debounce"

    self.prev_enabled = enabled
    self.prev_lead = hud_lead_visible

    return {
      "inferredApplyBrake": apply_brake,
      "inferredBasicAllowed": basic_allowed,
      "inferredBrakeAllowed": brake_allowed,
      "inferredBlockReason": reason,
      "inferredEngageBlockRemaining": block_remaining,
      "inferredCutInRemaining": cutin_remaining,
      "inferredCutInOK": cutin_ok,
      "inferredReqCounter": self.req_counter,
      "inferredReleaseCounter": self.release_counter,
      "inferredBrakeActive": self.brake_active,
      "inferredAboveOnThreshold": apply_brake > BRAKE_ON_THRESHOLD,
      "inferredBelowOffThreshold": apply_brake < BRAKE_OFF_THRESHOLD,
    }


def field_names():
  fields = [
    "t", "wallTime", "row", "services",
    "rxEvents", "rxFrames", "txEvents", "txFrames",

    "vEgo", "vEgoKph", "aEgo", "gas", "gasPressed",
    "brake", "brakePressed", "brakeLights", "standstill",
    "cruiseEnabled", "cruiseAvailable", "cruiseStandstill",
    "cruiseSpeed", "cruiseSpeedKph", "cruiseSpeedClusterKph",

    "carControlEnabled", "carControlActive", "hudLeadVisible",
    "actAccel", "actSpeed", "actSpeedKph", "actGas", "actBrake", "actSteer",

    "controlsEnabled", "controlsActive", "controlsATarget",
    "controlsVPid", "controlsVCruise", "longControlState", "forceDecel",

    "planHasLead", "planSource", "planFcw",
    "planSpeed0", "planSpeed5", "planSpeedEnd",
    "planAccel0", "planAccel5", "planAccelEnd", "planShouldStop",

    "leadStatus", "leadDRel", "leadYRel", "leadVRel",
    "leadVLead", "leadVLeadK", "leadALeadK", "leadALeadTau", "leadFcw",

    "steeringAngleDeg", "steeringRateDeg", "steeringTorque",
    "steeringTorqueEps", "steeringPressed", "steeringRateLimited",
    "steerRatioLive", "liveParametersValid",
    "latType", "latActive", "latOutput", "latSaturated",
    "desiredAngleDeg", "controllerAngleDeg", "angleErrorDeg",
    "latP", "latI", "latF",

    "inferredApplyBrake", "inferredBasicAllowed", "inferredBrakeAllowed",
    "inferredBlockReason", "inferredEngageBlockRemaining",
    "inferredCutInRemaining", "inferredCutInOK",
    "inferredReqCounter", "inferredReleaseCounter", "inferredBrakeActive",
    "inferredAboveOnThreshold", "inferredBelowOffThreshold",

    "tx271State", "tx271StateName", "tx271Active",
    "tx271PumpRaw", "tx271Pump", "tx271Magnitude", "tx271Counter",
    "tx271Checksum", "tx271ChecksumExpected", "tx271ChecksumValid",
    "tx273ChecksumValid", "tx274ChecksumValid",
  ]

  for prefix in ("rx", "tx"):
    for address in CAN_IDS:
      key = "%s%03X" % (prefix, address)
      fields += [key, key + "Bus", key + "AgeMs"]

  return fields


def main():
  signal.signal(signal.SIGTERM, request_stop)
  signal.signal(signal.SIGINT, request_stop)

  services = available_services()
  sm = messaging.SubMaster(services)
  can_sock = open_sub_socket("can")
  sendcan_sock = open_sub_socket("sendcan")

  stamp = datetime.now().strftime("%m%d_%H%M%S")
  path = os.path.join(OUTPUT_DIR, "dnga_v25e_combined_%s.csv.gz" % stamp)

  rx_cache = {}
  tx_cache = {}
  inference = V25EInference()
  start = time.monotonic()
  last_print = start
  row_number = 0
  fields = field_names()
  rk = Ratekeeper(RATE_HZ)

  print("DNGA v2.5e standalone brake + steering logger")
  print("No car files are modified.")
  print("Services: " + ", ".join(services))
  print("Output: " + path)

  try:
    with gzip.open(path, "wt", newline="") as output:
      writer = csv.DictWriter(output, fieldnames=fields, extrasaction="ignore")
      writer.writeheader()

      while not STOP_REQUESTED:
        now = time.monotonic()
        sm.update(0)

        rx_events, rx_frames = drain_can(can_sock, "can", rx_cache)
        tx_events, tx_frames = drain_can(sendcan_sock, "sendcan", tx_cache)

        cs = sm["carState"]
        cc = sm["carControl"]
        controls = sm["controlsState"]
        actuators = get_path(cc, "actuators", None)
        cruise = get_path(cs, "cruiseState", None)
        hud = get_path(cc, "hudControl", None)

        radar = sm["radarState"] if "radarState" in services else None
        lead_one = get_path(radar, "leadOne", None)
        plan = sm["longitudinalPlan"] if "longitudinalPlan" in services else None
        live = sm["liveParameters"] if "liveParameters" in services else None

        plan_speeds = get_path(plan, "speeds", [])
        plan_accels = get_path(plan, "accels", [])

        v_ego = to_float(get_path(cs, "vEgo", 0.0), 0.0)
        a_ego = to_float(get_path(cs, "aEgo", 0.0), 0.0)
        act_accel = to_float(get_path(actuators, "accel", 0.0), 0.0)
        enabled = to_bool(get_path(cc, "enabled", False))
        cruise_enabled = to_bool(get_path(cruise, "enabled", False))
        standstill = to_bool(get_path(cs, "standstill", False))
        gas_pressed = to_bool(get_path(cs, "gasPressed", False))
        brake_pressed = to_bool(get_path(cs, "brakePressed", False))
        hud_lead = to_bool(get_path(hud, "leadVisible", False))

        inferred = inference.update(
          now, enabled, cruise_enabled, standstill, v_ego,
          gas_pressed, brake_pressed, hud_lead, act_accel, a_ego)

        tx271_hex = tx_cache.get(0x271, {}).get("hex", "")
        decoded_271 = decode_271(tx271_hex)

        row = {
          "t": now - start,
          "wallTime": datetime.now().isoformat(),
          "row": row_number,
          "services": ",".join(services),
          "rxEvents": rx_events,
          "rxFrames": rx_frames,
          "txEvents": tx_events,
          "txFrames": tx_frames,

          "vEgo": v_ego,
          "vEgoKph": v_ego * 3.6,
          "aEgo": a_ego,
          "gas": to_float(get_path(cs, "gas", float("nan"))),
          "gasPressed": gas_pressed,
          "brake": to_float(get_path(cs, "brake", float("nan"))),
          "brakePressed": brake_pressed,
          "brakeLights": to_bool(get_path(cs, "brakeLights", False)),
          "standstill": standstill,
          "cruiseEnabled": cruise_enabled,
          "cruiseAvailable": to_bool(get_path(cruise, "available", False)),
          "cruiseStandstill": to_bool(get_path(cruise, "standstill", False)),
          "cruiseSpeed": to_float(get_path(cruise, "speed", float("nan"))),
          "cruiseSpeedKph": to_float(get_path(cruise, "speed", 0.0), 0.0) * 3.6,
          "cruiseSpeedClusterKph": to_float(
            get_path(cruise, "speedCluster", 0.0), 0.0) * 3.6,

          "carControlEnabled": enabled,
          "carControlActive": to_bool(get_path(cc, "active", False)),
          "hudLeadVisible": hud_lead,
          "actAccel": act_accel,
          "actSpeed": to_float(get_path(actuators, "speed", float("nan"))),
          "actSpeedKph": to_float(get_path(actuators, "speed", 0.0), 0.0) * 3.6,
          "actGas": to_float(get_path(actuators, "gas", float("nan"))),
          "actBrake": to_float(get_path(actuators, "brake", float("nan"))),
          "actSteer": to_float(get_path(actuators, "steer", float("nan"))),

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
          "planShouldStop": to_bool(get_path(plan, "shouldStop", False)),

          "leadStatus": to_bool(get_path(lead_one, "status", False)),
          "leadDRel": to_float(get_path(lead_one, "dRel", float("nan"))),
          "leadYRel": to_float(get_path(lead_one, "yRel", float("nan"))),
          "leadVRel": to_float(get_path(lead_one, "vRel", float("nan"))),
          "leadVLead": to_float(get_path(lead_one, "vLead", float("nan"))),
          "leadVLeadK": to_float(get_path(lead_one, "vLeadK", float("nan"))),
          "leadALeadK": to_float(get_path(lead_one, "aLeadK", float("nan"))),
          "leadALeadTau": to_float(get_path(lead_one, "aLeadTau", float("nan"))),
          "leadFcw": to_bool(get_path(lead_one, "fcw", False)),

          "steeringAngleDeg": to_float(get_path(cs, "steeringAngleDeg", float("nan"))),
          "steeringRateDeg": to_float(get_path(cs, "steeringRateDeg", float("nan"))),
          "steeringTorque": to_float(get_path(cs, "steeringTorque", float("nan"))),
          "steeringTorqueEps": to_float(get_path(cs, "steeringTorqueEps", float("nan"))),
          "steeringPressed": to_bool(get_path(cs, "steeringPressed", False)),
          "steeringRateLimited": to_bool(get_path(cs, "steeringRateLimited", False)),
          "steerRatioLive": to_float(get_path(live, "steerRatio", float("nan"))),
          "liveParametersValid": to_bool(get_path(live, "valid", False)),

          "tx273ChecksumValid": decode_checksum(
            0x273, tx_cache.get(0x273, {}).get("hex", "")),
          "tx274ChecksumValid": decode_checksum(
            0x274, tx_cache.get(0x274, {}).get("hex", "")),
        }

        row.update(lateral_values(controls))
        row.update(inferred)
        row.update(decoded_271)
        row.update(can_columns("rx", rx_cache, now))
        row.update(can_columns("tx", tx_cache, now))
        writer.writerow(row)

        row_number += 1
        if row_number % 100 == 0:
          output.flush()

        if now - last_print >= 5.0:
          print(
            "%.1fs rows=%d speed=%.1f actAccel=%.3f inferred=%s "
            "tx271=%s mag=%s reason=%s steer=%.2f" % (
              now - start,
              row_number,
              row["vEgoKph"],
              row["actAccel"],
              str(row["inferredBrakeActive"]),
              row["tx271StateName"],
              str(row["tx271Magnitude"]),
              row["inferredBlockReason"],
              row["actSteer"],
            ),
            flush=True,
          )
          last_print = now

        rk.keep_time()

  finally:
    pass

  print("Logger stopped cleanly.")
  print("Saved %d rows to:" % row_number)
  print(path)


if __name__ == "__main__":
  main()
