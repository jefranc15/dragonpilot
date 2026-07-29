#!/usr/bin/env python3
"""
Standalone DragonPilot 0.8.13 DNGA V2.5J longitudinal/CAN logger.

This script only subscribes to cereal services. It does not import, patch, or
modify any DNGA car files. It is intended for the Yaris Cross V2.5J
stock-scaled proportional-brake road test.
"""

import csv
import gzip
import json
import math
import os
import signal
import sys
import time
from collections import deque
from datetime import datetime

from cereal import messaging


LOGGER_VERSION = "2.5J-1.0-standalone"
OUTPUT_DIR = "/data/openpilot"
SUMMARY_PERIOD = 0.05
BASELINE_SECONDS = 5.0
EVENT_PRE_SECONDS = 2.0
EVENT_POST_SECONDS = 5.0

# Includes the longitudinal frames used by the earlier V2.5I logger, plus
# pedal, brake, wheel-speed, steering, and HUD frames needed to correlate the
# V2.5J road test. Unknown IDs are intentionally retained for comparison with
# the earlier mapping logs.
SELECTED_ADDRESSES = {
  0x0A1,  # BRAKE
  0x0A4,  # STEERING_MODULE
  0x18E,  # GAS_PEDAL
  0x18F,  # GAS_PEDAL_2
  0x1A0,  # WHEEL_SPEED
  0x1AB,  # BUTTONS
  0x1C0,  # EPS_SHAFT_TORQUE
  0x1D0,  # STEERING_LKAS
  0x1F0,  # prior mapping target
  0x207,  # PCM_BUTTONS_HYBRID
  0x208,  # PCM_BUTTONS / ACC_MAIN
  0x20C,  # TRANSMISSION
  0x260,  # WHEEL_SPEED_CLEAN
  0x270,  # ADAS_AEB
  0x271,  # ACC_BRAKE
  0x273,  # ACC_CMD_HUD
  0x274,  # LKAS_HUD
  0x277,  # prior mapping target
  0x280,  # prior mapping target
  0x342,  # BSM
  0x358,  # METER_CLUSTER
  0x384,  # HANDBRAKE
  0x490,  # ADAS_UNKNOWN
  0x4F3,  # FWD_CAM_HEARTBEAT
  0x520,  # prior logger target (hex address)
}

ADDRESS_NAMES = {
  0x0A1: "BRAKE",
  0x0A4: "STEERING_MODULE",
  0x18E: "GAS_PEDAL",
  0x18F: "GAS_PEDAL_2",
  0x1A0: "WHEEL_SPEED",
  0x1AB: "BUTTONS",
  0x1C0: "EPS_SHAFT_TORQUE",
  0x1D0: "STEERING_LKAS",
  0x207: "PCM_BUTTONS_HYBRID",
  0x208: "PCM_BUTTONS",
  0x20C: "TRANSMISSION",
  0x260: "WHEEL_SPEED_CLEAN",
  0x270: "ADAS_AEB",
  0x271: "ACC_BRAKE",
  0x273: "ACC_CMD_HUD",
  0x274: "LKAS_HUD",
  0x342: "BSM",
  0x358: "METER_CLUSTER",
  0x384: "HANDBRAKE",
  0x490: "ADAS_UNKNOWN",
  0x4F3: "FWD_CAM_HEARTBEAT",
}

STATE_NAMES_271 = {
  0x00: "disabled",
  0x01: "ready",
  0x21: "braking",
  0x30: "stop_hold_30",
  0x31: "stop_hold_31",
}

SUMMARY_FIELDS = [
  "t", "wallTime", "row", "loggerVersion",
  "startupPhase", "eventWindow", "eventId", "eventReasons",
  "rawFramesWritten", "selectedFramesWritten", "rxFrames", "txFrames",
  "carStateAlive", "carControlAlive", "controlsStateAlive",
  "radarStateAlive", "longitudinalPlanAlive", "pandaStatesAlive",
  "vEgo", "vEgoKph", "vEgoRaw", "aEgo", "estimatedJerk",
  "standstill", "gearShifter", "gas", "gasPressed",
  "brake", "brakePressed", "brakeLights",
  "cruiseAvailable", "cruiseEnabled", "cruiseStandstill", "cruiseSpeedKph",
  "carControlEnabled", "carControlActive",
  "controlsEnabled", "controlsActive", "longControlState",
  "actAccel", "actGas", "actBrake", "actSpeed",
  "controlsATarget", "planSource", "planShouldStop",
  "planSpeed0", "planSpeedEnd", "planAccel0", "planAccelEnd",
  "leadStatus", "leadDRel", "leadVRel", "leadVLead",
  "leadALeadK", "leadModelProb",
  "steeringAngleDeg", "steeringRateDeg", "steeringTorque",
  "steeringTorqueEps", "steeringPressed",
  "pandaCount", "pandaIgnitionLine", "pandaIgnitionCan",
  "pandaType", "pandaSafetyModel", "pandaSafetyParam",
  "pandaControlsAllowed", "pandaHeartbeatLost", "pandaHarnessStatus",
  "enableAgeSec", "brakeStartupGateRemainingSec",
  "actAccelMode", "recoveryActiveApprox",
  "v25jSpeedScale", "v25jTargetDecel",
  "tx271Raw", "tx271Bus", "tx271AgeMs",
  "tx271State", "tx271StateName", "tx271Active",
  "tx271PumpReactionRaw", "tx271PumpReaction",
  "tx271PumpStageRaw", "tx271PumpStage",
  "tx271DecelByte", "tx271DecelCmd", "tx271CombinedMagnitude",
  "tx271Counter", "tx271ChecksumValid", "tx271RampDelta",
  "tx271TargetError", "tx271StateChanged",
  "rx271Raw", "rx271Bus", "rx271AgeMs",
  "rx271State", "rx271StateName", "rx271Active",
  "rx271PumpReactionRaw", "rx271PumpReaction",
  "rx271PumpStageRaw", "rx271PumpStage",
  "rx271DecelByte", "rx271DecelCmd", "rx271CombinedMagnitude",
  "rx271Counter", "rx271ChecksumValid",
  "tx273Raw", "tx273Bus", "tx273AgeMs",
  "tx273SetSpeedKph", "tx273FollowDistance", "tx273Lead",
  "tx273AccCmdKph", "tx273IsAccel", "tx273IsDecel",
  "tx273ChecksumValid",
  "rx273Raw", "rx273Bus", "rx273AgeMs",
  "rx273SetSpeedKph", "rx273FollowDistance", "rx273Lead",
  "rx273AccCmdKph", "rx273IsAccel", "rx273IsDecel",
  "rx273ChecksumValid",
  "rx208Raw", "rx208Bus", "rx208AgeMs",
  "rx208AccMain", "rx208AccRdy", "rx208ResPlus",
  "rx208SetMinus", "rx208Cancel", "rx208PedalDepressed",
  "rx208Counter",
  "tx274Raw", "tx274Bus", "tx274AgeMs",
  "tx274LkasSet", "tx274LkasEngaged",
  "tx274HoldWarning", "tx274LdaOff", "tx274LdaAlert",
  "tx274AebAlarm", "tx274AebBrake", "tx274FrontDepart",
  "brakeEntryCount", "brakeReleaseCount", "rapidReentryCount",
  "brakeTransitionsLast3Sec", "brakeChatterFlag",
  "positiveAccelBrakeOverlap", "decelRequestWithout271",
]

SELECTED_FIELDS = [
  "sequence", "t", "wallTime", "direction", "address", "addressHex",
  "name", "bus", "dlc", "data",
]

RAW_FIELDS = [
  "sequence", "t", "wallTime", "eventId", "eventReasons", "direction",
  "address", "addressHex", "name", "bus", "dlc", "data",
]


running = True


def stop_handler(_signum, _frame):
  global running
  running = False


def safe_attr(obj, path, default=""):
  try:
    value = obj
    for part in path.split("."):
      value = getattr(value, part)
    return value
  except Exception:
    return default


def first_or_default(values, default=""):
  try:
    return values[0] if len(values) else default
  except Exception:
    return default


def last_or_default(values, default=""):
  try:
    return values[-1] if len(values) else default
  except Exception:
    return default


def finite_float(value, default=0.0):
  try:
    out = float(value)
    return out if math.isfinite(out) else default
  except Exception:
    return default


def bool_value(value):
  try:
    return bool(value)
  except Exception:
    return False


def enum_text(value):
  try:
    text = str(value)
    return text.split(".")[-1]
  except Exception:
    return ""


def signed_byte(value):
  return value - 256 if value >= 128 else value


def dbc_motorola_unsigned(dat, start_bit, length):
  """Decode an unsigned DBC @0 (Motorola) signal."""
  value = 0
  bit = start_bit
  for _ in range(length):
    byte_index = bit // 8
    if byte_index >= len(dat):
      return 0
    value = (value << 1) | ((dat[byte_index] >> (bit % 8)) & 1)
    bit = bit + 15 if (bit % 8) == 0 else bit - 1
  return value


def dnga_checksum(address, dat_without_checksum):
  return (address + len(dat_without_checksum) + 1 + 2 +
          sum(dat_without_checksum)) & 0xFF


def decode_271(dat):
  out = {
    "raw": dat.hex().upper(),
    "state": "",
    "stateName": "",
    "active": False,
    "pumpReactionRaw": "",
    "pumpReaction": "",
    "pumpStageRaw": "",
    "pumpStage": "",
    "decelByte": "",
    "decelCmd": "",
    "combinedMagnitude": "",
    "counter": "",
    "checksumValid": False,
  }
  if len(dat) < 8:
    return out

  state = dat[1]
  pump_reaction_raw = signed_byte(dat[3])
  pump_stage_raw = dat[4]
  decel_byte = dat[5]
  active = state == 0x21

  out.update({
    "state": state,
    "stateName": STATE_NAMES_271.get(state, "unknown_%02X" % state),
    "active": active,
    "pumpReactionRaw": pump_reaction_raw,
    "pumpReaction": pump_reaction_raw * 0.1,
    "pumpStageRaw": pump_stage_raw,
    "pumpStage": pump_stage_raw * 0.1,
    "decelByte": decel_byte,
    "decelCmd": max(0.0, (200 - decel_byte) * 0.01) if active else 0.0,
    "combinedMagnitude": (pump_stage_raw << 8) | decel_byte,
    "counter": (dat[6] >> 2) & 0x7,
    "checksumValid": dnga_checksum(0x271, dat[:-1]) == dat[-1],
  })
  return out


def decode_273(dat):
  out = {
    "raw": dat.hex().upper(),
    "setSpeedKph": "",
    "followDistance": "",
    "lead": "",
    "accCmdKph": "",
    "isAccel": "",
    "isDecel": "",
    "checksumValid": False,
  }
  if len(dat) < 8:
    return out

  out.update({
    "setSpeedKph": dbc_motorola_unsigned(dat, 7, 8),
    "followDistance": dbc_motorola_unsigned(dat, 15, 2),
    "lead": dbc_motorola_unsigned(dat, 11, 1),
    "accCmdKph": dbc_motorola_unsigned(dat, 23, 16) * 0.01,
    "isAccel": dbc_motorola_unsigned(dat, 38, 1),
    "isDecel": dbc_motorola_unsigned(dat, 37, 1),
    "checksumValid": dnga_checksum(0x273, dat[:-1]) == dat[-1],
  })
  return out


def decode_208(dat):
  out = {
    "raw": dat.hex().upper(),
    "accMain": "",
    "accRdy": "",
    "resPlus": "",
    "setMinus": "",
    "cancel": "",
    "pedalDepressed": "",
    "counter": "",
  }
  if len(dat) < 6:
    return out

  out.update({
    "accMain": dbc_motorola_unsigned(dat, 7, 1),
    "accRdy": dbc_motorola_unsigned(dat, 9, 1),
    "resPlus": dbc_motorola_unsigned(dat, 6, 1),
    "setMinus": dbc_motorola_unsigned(dat, 5, 1),
    "cancel": dbc_motorola_unsigned(dat, 4, 1),
    "pedalDepressed": dbc_motorola_unsigned(dat, 12, 1),
    # This field is little-endian in the current DBC.
    "counter": dat[3] & 0x0F,
  })
  return out


def decode_274(dat):
  out = {
    "raw": dat.hex().upper(),
    "lkasSet": "",
    "lkasEngaged": "",
    "holdWarning": "",
    "ldaOff": "",
    "ldaAlert": "",
    "aebAlarm": "",
    "aebBrake": "",
    "frontDepart": "",
  }
  if len(dat) < 8:
    return out

  out.update({
    "lkasSet": dbc_motorola_unsigned(dat, 19, 1),
    "lkasEngaged": dbc_motorola_unsigned(dat, 30, 1),
    "holdWarning": dbc_motorola_unsigned(dat, 18, 1),
    "ldaOff": dbc_motorola_unsigned(dat, 13, 1),
    "ldaAlert": dbc_motorola_unsigned(dat, 14, 1),
    "aebAlarm": dbc_motorola_unsigned(dat, 8, 1),
    "aebBrake": dbc_motorola_unsigned(dat, 23, 1),
    "frontDepart": dbc_motorola_unsigned(dat, 9, 1),
  })
  return out


def decode_cached(cache, direction, address, decoder, now):
  item = cache.get((direction, address))
  if item is None:
    return {}, "", ""
  decoded = decoder(item["data"])
  return decoded, item["bus"], max(0.0, (now - item["time"]) * 1000.0)


def alive(sm, service):
  try:
    return bool(sm.alive[service])
  except Exception:
    return False


def service(sm, name):
  try:
    return sm[name]
  except Exception:
    return None


def make_frame(sequence, elapsed, direction, address, bus, dat):
  return {
    "sequence": sequence,
    "t": "%.6f" % elapsed,
    "wallTime": datetime.now().isoformat(timespec="milliseconds"),
    "direction": direction,
    "address": address,
    "addressHex": "0x%03X" % address,
    "name": ADDRESS_NAMES.get(address, ""),
    "bus": bus,
    "dlc": len(dat),
    "data": bytes(dat),
  }


def csv_frame_row(frame):
  row = dict(frame)
  row["data"] = frame["data"].hex().upper()
  return row


class EventCapture:
  def __init__(self, writer, start_time):
    self.writer = writer
    self.start_time = start_time
    self.ring = deque()
    self.event_id = 0
    self.reasons = set(["startup_baseline"])
    self.event_end = start_time + BASELINE_SECONDS
    self.last_written_sequence = 0
    self.written = 0

  def active(self, now):
    return now <= self.event_end

  def phase(self, now):
    return "BASELINE" if (now - self.start_time) < BASELINE_SECONDS else "DRIVE"

  def reason_text(self):
    return "|".join(sorted(self.reasons))

  def add_frame(self, frame, now):
    self.ring.append((now, frame))
    cutoff = now - EVENT_PRE_SECONDS
    while self.ring and self.ring[0][0] < cutoff:
      self.ring.popleft()

    if self.active(now):
      self.write_frame(frame)

  def write_frame(self, frame):
    if frame["sequence"] <= self.last_written_sequence:
      return
    row = csv_frame_row(frame)
    row["eventId"] = self.event_id
    row["eventReasons"] = self.reason_text()
    self.writer.writerow(row)
    self.last_written_sequence = frame["sequence"]
    self.written += 1

  def trigger(self, reason, now):
    if self.active(now):
      self.reasons.add(reason)
      self.event_end = max(self.event_end, now + EVENT_POST_SECONDS)
      return

    self.event_id += 1
    self.reasons = set([reason])
    self.event_end = now + EVENT_POST_SECONDS
    for _, frame in self.ring:
      self.write_frame(frame)


class V25JLogger:
  def __init__(self):
    self.start_time = time.monotonic()
    self.stamp = datetime.now().strftime("%m%d_%H%M%S")
    self.summary_path = os.path.join(
      OUTPUT_DIR, "dnga_v25j_summary_%s.csv.gz" % self.stamp)
    self.selected_path = os.path.join(
      OUTPUT_DIR, "dnga_v25j_selected_can_%s.csv.gz" % self.stamp)
    self.raw_path = os.path.join(
      OUTPUT_DIR, "dnga_v25j_raw_events_%s.csv.gz" % self.stamp)

    self.summary_file = gzip.open(
      self.summary_path, "wt", newline="", compresslevel=1)
    self.selected_file = gzip.open(
      self.selected_path, "wt", newline="", compresslevel=1)
    self.raw_file = gzip.open(
      self.raw_path, "wt", newline="", compresslevel=1)
    self.summary_writer = csv.DictWriter(
      self.summary_file, fieldnames=SUMMARY_FIELDS, extrasaction="ignore")
    self.selected_writer = csv.DictWriter(
      self.selected_file, fieldnames=SELECTED_FIELDS, extrasaction="ignore")
    self.raw_writer = csv.DictWriter(
      self.raw_file, fieldnames=RAW_FIELDS, extrasaction="ignore")
    self.summary_writer.writeheader()
    self.selected_writer.writeheader()
    self.raw_writer.writeheader()

    self.events = EventCapture(self.raw_writer, self.start_time)
    self.cache = {}
    self.sequence = 0
    self.row = 0
    self.selected_written = 0
    self.rx_frames = 0
    self.tx_frames = 0

    self.prev_a_ego = None
    self.prev_a_ego_time = None
    self.prev_act_mode = "neutral"
    self.prev_lead_status = None
    self.prev_gas_pressed = None
    self.prev_brake_pressed = None
    self.prev_tx271_state = None
    self.prev_tx271_decel = None
    self.last_brake_release_time = None
    self.brake_entry_count = 0
    self.brake_release_count = 0
    self.rapid_reentry_count = 0
    self.brake_transition_times = deque()
    self.last_meaningful_decel_time = -1e9
    self.enable_since = None
    self.was_enabled = False

    try:
      with open(os.path.join(OUTPUT_DIR, ".dnga_v25j_last_stamp"), "w") as f:
        f.write(self.stamp + "\n")
    except Exception:
      pass

  def close(self):
    for f in (self.summary_file, self.selected_file, self.raw_file):
      try:
        f.flush()
        f.close()
      except Exception:
        pass

  def flush(self):
    for f in (self.summary_file, self.selected_file, self.raw_file):
      try:
        f.flush()
      except Exception:
        pass

  def record_packet(self, packet, direction, now):
    try:
      frames = packet.can if direction == "rx" else packet.sendcan
    except Exception:
      return

    for can_frame in frames:
      try:
        address = int(can_frame.address)
        bus = int(can_frame.src)
        dat = bytes(can_frame.dat)
      except Exception:
        continue

      self.sequence += 1
      elapsed = now - self.start_time
      frame = make_frame(
        self.sequence, elapsed, direction, address, bus, dat)

      if direction == "rx":
        self.rx_frames += 1
      else:
        self.tx_frames += 1

      self.cache[(direction, address)] = {
        "time": now,
        "bus": bus,
        "data": dat,
      }

      if address in SELECTED_ADDRESSES:
        self.selected_writer.writerow(csv_frame_row(frame))
        self.selected_written += 1

      self.events.add_frame(frame, now)

      if direction == "tx" and address == 0x271:
        decoded = decode_271(dat)
        state = decoded.get("state", "")
        decel = finite_float(decoded.get("decelCmd", 0.0))
        if self.prev_tx271_state is not None and state != self.prev_tx271_state:
          self.events.trigger(
            "tx271_%02X_to_%02X" % (self.prev_tx271_state, state), now)
          self.brake_transition_times.append(now)
          if state == 0x21:
            self.brake_entry_count += 1
            if (self.last_brake_release_time is not None and
                now - self.last_brake_release_time <= 2.0):
              self.rapid_reentry_count += 1
              self.events.trigger("rapid_brake_reentry", now)
          elif self.prev_tx271_state == 0x21:
            self.brake_release_count += 1
            self.last_brake_release_time = now
        self.prev_tx271_state = state

        if self.prev_tx271_decel is not None:
          delta = decel - self.prev_tx271_decel
          if abs(delta) > 0.061:
            self.events.trigger("tx271_ramp_jump", now)
        self.prev_tx271_decel = decel

  def summary(self, sm, now):
    elapsed = now - self.start_time
    car_state = service(sm, "carState")
    car_control = service(sm, "carControl")
    controls_state = service(sm, "controlsState")
    radar_state = service(sm, "radarState")
    long_plan = service(sm, "longitudinalPlan")
    panda_states = service(sm, "pandaStates")

    v_ego = finite_float(safe_attr(car_state, "vEgo", 0.0))
    a_ego = finite_float(safe_attr(car_state, "aEgo", 0.0))
    act_accel = finite_float(
      safe_attr(car_control, "actuators.accel", 0.0))
    gas_pressed = bool_value(safe_attr(car_state, "gasPressed", False))
    brake_pressed = bool_value(safe_attr(car_state, "brakePressed", False))
    standstill = bool_value(safe_attr(car_state, "standstill", False))
    cruise_enabled = bool_value(
      safe_attr(car_state, "cruiseState.enabled", False))
    cc_enabled = bool_value(safe_attr(car_control, "enabled", False))
    controls_enabled = bool_value(
      safe_attr(controls_state, "enabled", False))
    enabled = cc_enabled or controls_enabled

    if enabled and not self.was_enabled:
      self.enable_since = now
    elif not enabled:
      self.enable_since = None
    self.was_enabled = enabled
    enable_age = (now - self.enable_since) if self.enable_since else 0.0

    if self.prev_a_ego is not None and self.prev_a_ego_time is not None:
      dt = max(1e-3, now - self.prev_a_ego_time)
      estimated_jerk = (a_ego - self.prev_a_ego) / dt
    else:
      estimated_jerk = 0.0
    self.prev_a_ego = a_ego
    self.prev_a_ego_time = now

    if act_accel < -0.05:
      act_mode = "brake"
    elif act_accel > 0.05:
      act_mode = "propel"
    else:
      act_mode = "neutral"

    if act_accel < -0.18:
      self.last_meaningful_decel_time = now
    recovery_approx = (
      act_accel > 0.0 and
      now - self.last_meaningful_decel_time < 1.2
    )

    lead = safe_attr(radar_state, "leadOne", None)
    lead_status = bool_value(safe_attr(lead, "status", False))

    if self.prev_act_mode != act_mode:
      if set([self.prev_act_mode, act_mode]) == set(["brake", "propel"]):
        self.events.trigger(
          "actAccel_%s_to_%s" % (self.prev_act_mode, act_mode), now)
      self.prev_act_mode = act_mode

    if self.prev_lead_status is not None and lead_status != self.prev_lead_status:
      self.events.trigger(
        "lead_%s" % ("appeared" if lead_status else "lost"), now)
    self.prev_lead_status = lead_status

    if self.prev_gas_pressed is not None and gas_pressed != self.prev_gas_pressed:
      self.events.trigger(
        "driver_gas_%s" % ("on" if gas_pressed else "off"), now)
    if (self.prev_brake_pressed is not None and
        brake_pressed != self.prev_brake_pressed):
      self.events.trigger(
        "driver_brake_%s" % ("on" if brake_pressed else "off"), now)
    self.prev_gas_pressed = gas_pressed
    self.prev_brake_pressed = brake_pressed

    if abs(estimated_jerk) >= 1.5 and v_ego > 1.0:
      self.events.trigger("measured_jerk", now)

    speed_scale = 1.0
    if v_ego > 0.0:
      speed_scale = max(
        1.0 / 1.5,
        1.0 - (v_ego / (140.0 / 3.6)) * (1.0 - 1.0 / 1.5),
      )

    brake_gate_remaining = max(0.0, 2.0 - enable_age) if enabled else 0.0
    brake_allowed_approx = (
      enabled and cruise_enabled and not standstill and
      enable_age > 2.0 and v_ego > 1.0 and act_accel < 0.0 and
      not gas_pressed and not brake_pressed
    )
    target_decel = (
      min(1.09, max(0.0, -act_accel * speed_scale))
      if brake_allowed_approx else 0.0
    )
    if target_decel < 0.05:
      target_decel = 0.0

    tx271, tx271_bus, tx271_age = decode_cached(
      self.cache, "tx", 0x271, decode_271, now)
    rx271, rx271_bus, rx271_age = decode_cached(
      self.cache, "rx", 0x271, decode_271, now)
    tx273, tx273_bus, tx273_age = decode_cached(
      self.cache, "tx", 0x273, decode_273, now)
    rx273, rx273_bus, rx273_age = decode_cached(
      self.cache, "rx", 0x273, decode_273, now)
    rx208, rx208_bus, rx208_age = decode_cached(
      self.cache, "rx", 0x208, decode_208, now)
    tx274, tx274_bus, tx274_age = decode_cached(
      self.cache, "tx", 0x274, decode_274, now)

    tx271_active = bool_value(tx271.get("active", False))
    tx271_decel = finite_float(tx271.get("decelCmd", 0.0))
    positive_overlap = tx271_active and act_accel > 0.01
    decel_without_271 = (
      brake_allowed_approx and target_decel >= 0.05 and
      not tx271_active and tx271_age < 150.0
    )
    if positive_overlap:
      self.events.trigger("positive_accel_with_active_271", now)
    if decel_without_271:
      self.events.trigger("decel_request_without_271", now)

    while self.brake_transition_times and (
        now - self.brake_transition_times[0] > 3.0):
      self.brake_transition_times.popleft()
    chatter_flag = len(self.brake_transition_times) >= 4
    if chatter_flag:
      self.events.trigger("brake_state_chatter", now)

    tx271_state_changed = False
    if tx271.get("state", "") != "":
      tx271_state_changed = (
        getattr(self, "_last_summary_tx271_state", None) is not None and
        tx271.get("state") != self._last_summary_tx271_state
      )
      self._last_summary_tx271_state = tx271.get("state")

    tx271_ramp_delta = 0.0
    if tx271_active:
      last = getattr(self, "_last_summary_tx271_decel", None)
      if last is not None:
        tx271_ramp_delta = tx271_decel - last
      self._last_summary_tx271_decel = tx271_decel
    else:
      self._last_summary_tx271_decel = 0.0

    plan_speeds = safe_attr(long_plan, "speeds", [])
    plan_accels = safe_attr(long_plan, "accels", [])

    panda_count = 0
    panda = None
    try:
      panda_count = len(panda_states)
      panda = panda_states[0] if panda_count else None
    except Exception:
      pass

    row = {
      "t": "%.6f" % elapsed,
      "wallTime": datetime.now().isoformat(timespec="milliseconds"),
      "row": self.row,
      "loggerVersion": LOGGER_VERSION,
      "startupPhase": self.events.phase(now),
      "eventWindow": self.events.active(now),
      "eventId": self.events.event_id if self.events.active(now) else "",
      "eventReasons": self.events.reason_text() if self.events.active(now) else "",
      "rawFramesWritten": self.events.written,
      "selectedFramesWritten": self.selected_written,
      "rxFrames": self.rx_frames,
      "txFrames": self.tx_frames,
      "carStateAlive": alive(sm, "carState"),
      "carControlAlive": alive(sm, "carControl"),
      "controlsStateAlive": alive(sm, "controlsState"),
      "radarStateAlive": alive(sm, "radarState"),
      "longitudinalPlanAlive": alive(sm, "longitudinalPlan"),
      "pandaStatesAlive": alive(sm, "pandaStates"),
      "vEgo": v_ego,
      "vEgoKph": v_ego * 3.6,
      "vEgoRaw": safe_attr(car_state, "vEgoRaw", ""),
      "aEgo": a_ego,
      "estimatedJerk": estimated_jerk,
      "standstill": standstill,
      "gearShifter": enum_text(safe_attr(car_state, "gearShifter", "")),
      "gas": safe_attr(car_state, "gas", ""),
      "gasPressed": gas_pressed,
      "brake": safe_attr(car_state, "brake", ""),
      "brakePressed": brake_pressed,
      "brakeLights": safe_attr(car_state, "brakeLights", ""),
      "cruiseAvailable": safe_attr(
        car_state, "cruiseState.available", ""),
      "cruiseEnabled": cruise_enabled,
      "cruiseStandstill": safe_attr(
        car_state, "cruiseState.standstill", ""),
      "cruiseSpeedKph": finite_float(
        safe_attr(car_state, "cruiseState.speed", 0.0)) * 3.6,
      "carControlEnabled": cc_enabled,
      "carControlActive": safe_attr(car_control, "active", ""),
      "controlsEnabled": controls_enabled,
      "controlsActive": safe_attr(controls_state, "active", ""),
      "longControlState": enum_text(
        safe_attr(controls_state, "longControlState", "")),
      "actAccel": act_accel,
      "actGas": safe_attr(car_control, "actuators.gas", ""),
      "actBrake": safe_attr(car_control, "actuators.brake", ""),
      "actSpeed": safe_attr(car_control, "actuators.speed", ""),
      "controlsATarget": safe_attr(controls_state, "aTarget", ""),
      "planSource": enum_text(
        safe_attr(long_plan, "longitudinalPlanSource",
                  safe_attr(long_plan, "source", ""))),
      "planShouldStop": safe_attr(long_plan, "shouldStop", ""),
      "planSpeed0": first_or_default(plan_speeds),
      "planSpeedEnd": last_or_default(plan_speeds),
      "planAccel0": first_or_default(plan_accels),
      "planAccelEnd": last_or_default(plan_accels),
      "leadStatus": lead_status,
      "leadDRel": safe_attr(lead, "dRel", ""),
      "leadVRel": safe_attr(lead, "vRel", ""),
      "leadVLead": safe_attr(lead, "vLead", ""),
      "leadALeadK": safe_attr(lead, "aLeadK", ""),
      "leadModelProb": safe_attr(lead, "modelProb", ""),
      "steeringAngleDeg": safe_attr(car_state, "steeringAngleDeg", ""),
      "steeringRateDeg": safe_attr(car_state, "steeringRateDeg", ""),
      "steeringTorque": safe_attr(car_state, "steeringTorque", ""),
      "steeringTorqueEps": safe_attr(car_state, "steeringTorqueEps", ""),
      "steeringPressed": safe_attr(car_state, "steeringPressed", ""),
      "pandaCount": panda_count,
      "pandaIgnitionLine": safe_attr(panda, "ignitionLine", ""),
      "pandaIgnitionCan": safe_attr(panda, "ignitionCan", ""),
      "pandaType": enum_text(safe_attr(panda, "pandaType", "")),
      "pandaSafetyModel": enum_text(safe_attr(panda, "safetyModel", "")),
      "pandaSafetyParam": safe_attr(panda, "safetyParam", ""),
      "pandaControlsAllowed": safe_attr(panda, "controlsAllowed", ""),
      "pandaHeartbeatLost": safe_attr(panda, "heartbeatLost", ""),
      "pandaHarnessStatus": enum_text(
        safe_attr(panda, "harnessStatus", "")),
      "enableAgeSec": enable_age,
      "brakeStartupGateRemainingSec": brake_gate_remaining,
      "actAccelMode": act_mode,
      "recoveryActiveApprox": recovery_approx,
      "v25jSpeedScale": speed_scale,
      "v25jTargetDecel": target_decel,
      "tx271Raw": tx271.get("raw", ""),
      "tx271Bus": tx271_bus,
      "tx271AgeMs": tx271_age,
      "tx271State": tx271.get("state", ""),
      "tx271StateName": tx271.get("stateName", ""),
      "tx271Active": tx271_active,
      "tx271PumpReactionRaw": tx271.get("pumpReactionRaw", ""),
      "tx271PumpReaction": tx271.get("pumpReaction", ""),
      "tx271PumpStageRaw": tx271.get("pumpStageRaw", ""),
      "tx271PumpStage": tx271.get("pumpStage", ""),
      "tx271DecelByte": tx271.get("decelByte", ""),
      "tx271DecelCmd": tx271.get("decelCmd", ""),
      "tx271CombinedMagnitude": tx271.get("combinedMagnitude", ""),
      "tx271Counter": tx271.get("counter", ""),
      "tx271ChecksumValid": tx271.get("checksumValid", ""),
      "tx271RampDelta": tx271_ramp_delta,
      "tx271TargetError": (
        tx271_decel - target_decel if tx271_active else -target_decel),
      "tx271StateChanged": tx271_state_changed,
      "rx271Raw": rx271.get("raw", ""),
      "rx271Bus": rx271_bus,
      "rx271AgeMs": rx271_age,
      "rx271State": rx271.get("state", ""),
      "rx271StateName": rx271.get("stateName", ""),
      "rx271Active": rx271.get("active", ""),
      "rx271PumpReactionRaw": rx271.get("pumpReactionRaw", ""),
      "rx271PumpReaction": rx271.get("pumpReaction", ""),
      "rx271PumpStageRaw": rx271.get("pumpStageRaw", ""),
      "rx271PumpStage": rx271.get("pumpStage", ""),
      "rx271DecelByte": rx271.get("decelByte", ""),
      "rx271DecelCmd": rx271.get("decelCmd", ""),
      "rx271CombinedMagnitude": rx271.get("combinedMagnitude", ""),
      "rx271Counter": rx271.get("counter", ""),
      "rx271ChecksumValid": rx271.get("checksumValid", ""),
      "tx273Raw": tx273.get("raw", ""),
      "tx273Bus": tx273_bus,
      "tx273AgeMs": tx273_age,
      "tx273SetSpeedKph": tx273.get("setSpeedKph", ""),
      "tx273FollowDistance": tx273.get("followDistance", ""),
      "tx273Lead": tx273.get("lead", ""),
      "tx273AccCmdKph": tx273.get("accCmdKph", ""),
      "tx273IsAccel": tx273.get("isAccel", ""),
      "tx273IsDecel": tx273.get("isDecel", ""),
      "tx273ChecksumValid": tx273.get("checksumValid", ""),
      "rx273Raw": rx273.get("raw", ""),
      "rx273Bus": rx273_bus,
      "rx273AgeMs": rx273_age,
      "rx273SetSpeedKph": rx273.get("setSpeedKph", ""),
      "rx273FollowDistance": rx273.get("followDistance", ""),
      "rx273Lead": rx273.get("lead", ""),
      "rx273AccCmdKph": rx273.get("accCmdKph", ""),
      "rx273IsAccel": rx273.get("isAccel", ""),
      "rx273IsDecel": rx273.get("isDecel", ""),
      "rx273ChecksumValid": rx273.get("checksumValid", ""),
      "rx208Raw": rx208.get("raw", ""),
      "rx208Bus": rx208_bus,
      "rx208AgeMs": rx208_age,
      "rx208AccMain": rx208.get("accMain", ""),
      "rx208AccRdy": rx208.get("accRdy", ""),
      "rx208ResPlus": rx208.get("resPlus", ""),
      "rx208SetMinus": rx208.get("setMinus", ""),
      "rx208Cancel": rx208.get("cancel", ""),
      "rx208PedalDepressed": rx208.get("pedalDepressed", ""),
      "rx208Counter": rx208.get("counter", ""),
      "tx274Raw": tx274.get("raw", ""),
      "tx274Bus": tx274_bus,
      "tx274AgeMs": tx274_age,
      "tx274LkasSet": tx274.get("lkasSet", ""),
      "tx274LkasEngaged": tx274.get("lkasEngaged", ""),
      "tx274HoldWarning": tx274.get("holdWarning", ""),
      "tx274LdaOff": tx274.get("ldaOff", ""),
      "tx274LdaAlert": tx274.get("ldaAlert", ""),
      "tx274AebAlarm": tx274.get("aebAlarm", ""),
      "tx274AebBrake": tx274.get("aebBrake", ""),
      "tx274FrontDepart": tx274.get("frontDepart", ""),
      "brakeEntryCount": self.brake_entry_count,
      "brakeReleaseCount": self.brake_release_count,
      "rapidReentryCount": self.rapid_reentry_count,
      "brakeTransitionsLast3Sec": len(self.brake_transition_times),
      "brakeChatterFlag": chatter_flag,
      "positiveAccelBrakeOverlap": positive_overlap,
      "decelRequestWithout271": decel_without_271,
    }
    self.summary_writer.writerow(row)
    self.row += 1


def drain_socket(sock, direction, logger, now):
  try:
    packets = messaging.drain_sock(sock, wait_for_one=False)
  except TypeError:
    packets = messaging.drain_sock(sock)
  except Exception:
    return
  for packet in packets:
    logger.record_packet(packet, direction, now)


def main():
  os.makedirs(OUTPUT_DIR, exist_ok=True)

  services = [
    "carState",
    "carControl",
    "controlsState",
    "radarState",
    "longitudinalPlan",
    "pandaStates",
  ]
  sm = messaging.SubMaster(services)
  can_sock = messaging.sub_sock("can", conflate=False, timeout=20)
  sendcan_sock = messaging.sub_sock("sendcan", conflate=False, timeout=20)

  logger = V25JLogger()
  print("DNGA V2.5J standalone logger started")
  print("summary:  %s" % logger.summary_path)
  print("selected: %s" % logger.selected_path)
  print("raw:      %s" % logger.raw_path)
  sys.stdout.flush()

  next_summary = time.monotonic()
  next_flush = next_summary + 1.0
  next_status = next_summary + 10.0

  try:
    while running:
      now = time.monotonic()
      sm.update(0)
      drain_socket(can_sock, "rx", logger, now)
      drain_socket(sendcan_sock, "tx", logger, now)

      if now >= next_summary:
        logger.summary(sm, now)
        while next_summary <= now:
          next_summary += SUMMARY_PERIOD

      if now >= next_flush:
        logger.flush()
        next_flush = now + 1.0

      if now >= next_status:
        print(
          "running %.0fs rows=%d selected=%d raw=%d events=%d" %
          (now - logger.start_time, logger.row, logger.selected_written,
           logger.events.written, logger.events.event_id)
        )
        sys.stdout.flush()
        next_status = now + 10.0

      time.sleep(0.005)
  finally:
    logger.close()
    print("DNGA V2.5J logger stopped cleanly")
    print("Upload these three files:")
    print(logger.summary_path)
    print(logger.selected_path)
    print(logger.raw_path)
    sys.stdout.flush()


if __name__ == "__main__":
  signal.signal(signal.SIGTERM, stop_handler)
  signal.signal(signal.SIGINT, stop_handler)
  main()
