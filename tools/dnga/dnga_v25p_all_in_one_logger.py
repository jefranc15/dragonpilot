#!/usr/bin/env python3
"""
DNGA V2.5P all-in-one drive logger.

Logs, on one CSV timeline:
  * planner acceleration and target-speed data
  * likely curve-deceleration signals / source
  * path curvature and estimated lateral acceleration
  * lead information
  * requested, speed-limited, and actually transmitted steering torque
  * driver/EPS steering torque and steering-limit bounds
  * outgoing 0x1D0, 0x271, and 0x273 raw frames and decoded fields

This does not replace rlog.bz2. Keep the rlog for full-fidelity analysis.
Designed to run alongside DragonPilot/openpilot without modifying the control loop.
"""

import csv
import math
import os
import signal
import sys
import time
from datetime import datetime

import cereal.messaging as messaging
from cereal import car
from cereal.services import service_list
from common.numpy_fast import clip, interp
from common.params import Params

try:
  from common.realtime import Ratekeeper
except ImportError:
  Ratekeeper = None


LOG_HZ = 50
DEFAULT_LOG_DIR = "/data/openpilot/dnga_logs"

WANTED_SERVICES = [
  "carState",
  "carControl",
  "controlsState",
  "longitudinalPlan",
  "lateralPlan",
  "radarState",
  "modelV2",
  "liveParameters",
  "sendcan",
]


def safe_get(obj, name, default=None):
  try:
    return getattr(obj, name)
  except Exception:
    return default


def first_existing(obj, names, default=None):
  for name in names:
    value = safe_get(obj, name, None)
    if value is not None:
      return value
  return default


def nested_get(obj, path, default=None):
  cur = obj
  for name in path.split("."):
    cur = safe_get(cur, name, None)
    if cur is None:
      return default
  return cur


def first_item(value, default=math.nan, index=0):
  try:
    return value[index]
  except Exception:
    return default


def enum_text(value):
  if value is None:
    return ""
  try:
    return str(value)
  except Exception:
    return ""


def finite(value, default=math.nan):
  try:
    value = float(value)
    return value if math.isfinite(value) else default
  except Exception:
    return default


def bool_int(value):
  try:
    return 1 if bool(value) else 0
  except Exception:
    return 0


def signed_u8(value):
  return value - 256 if value >= 128 else value


def decode_steering_lkas(dat):
  """Decode STEER_CMD from DBC signal 7|11@0-; return commanded apply_steer."""
  if dat is None or len(dat) < 2:
    return math.nan
  raw = (dat[0] << 3) | (dat[1] >> 5)
  if raw & 0x400:
    raw -= 0x800
  # dngacan packs STEER_CMD = -apply_steer
  return float(-raw)


def decode_acc_brake(dat):
  if dat is None or len(dat) < 8:
    return {}
  magnitude = (dat[4] << 8) | dat[5]
  return {
    "tx_271_hex": dat.hex(),
    "tx_271_brake_state": dat[1],
    "tx_271_unknown_byte2": dat[2],
    "tx_271_pump_reaction2": signed_u8(dat[3]) * 0.1,
    "tx_271_pump_positive": dat[4] * 0.1,
    "tx_271_magnitude_raw": magnitude,
    "tx_271_decel_cmd": max(0.0, 2.0 - dat[5] * 0.01),
  }


def decode_acc_cmd(dat):
  if dat is None or len(dat) < 8:
    return {}
  return {
    "tx_273_hex": dat.hex(),
    "tx_273_set_speed_kph": dat[0],
    "tx_273_acc_cmd_mps": ((dat[2] << 8) | dat[3]) * 0.01,
    "tx_273_state_byte4": dat[4],
    "tx_273_is_accel": (dat[4] >> 6) & 1,
    "tx_273_is_decel": (dat[4] >> 5) & 1,
  }


def get_latest_tx_frames(sendcan):
  frames = {}
  try:
    for msg in sendcan:
      addr = int(msg.address)
      if addr in (0x1D0, 0x271, 0x273):
        frames[addr] = {
          "dat": bytes(msg.dat),
          "src": int(safe_get(msg, "src", -1)),
        }
  except Exception:
    pass
  return frames


def estimate_model_curvature(model):
  """Estimate near-path curvature from the first 3 valid model position points."""
  try:
    xs = list(model.position.x)
    ys = list(model.position.y)
    if len(xs) < 3 or len(ys) < 3:
      return math.nan

    # Use separated near-field points to reduce quantization/noise.
    i0 = 0
    i1 = min(5, len(xs) - 2)
    i2 = min(10, len(xs) - 1)
    x1, y1 = float(xs[i0]), float(ys[i0])
    x2, y2 = float(xs[i1]), float(ys[i1])
    x3, y3 = float(xs[i2]), float(ys[i2])

    a = math.hypot(x2 - x1, y2 - y1)
    b = math.hypot(x3 - x2, y3 - y2)
    c = math.hypot(x3 - x1, y3 - y1)
    denom = a * b * c
    if denom < 1e-6:
      return 0.0

    twice_area = ((x2 - x1) * (y3 - y1)) - ((y2 - y1) * (x3 - x1))
    return 2.0 * twice_area / denom
  except Exception:
    return math.nan


def load_car_params():
  raw = Params().get("CarParams", block=True)
  if not raw:
    raise RuntimeError("CarParams is unavailable. Start the car/openpilot first.")
  cp = car.CarParams.from_bytes(raw)
  torque_bp = list(cp.lateralParams.torqueBP)
  torque_v = list(cp.lateralParams.torqueV)
  return cp, torque_bp, torque_v


def choose_services():
  available = []
  missing = []
  for service in WANTED_SERVICES:
    if service in service_list:
      available.append(service)
    else:
      missing.append(service)
  required = {"carState", "carControl", "controlsState", "sendcan"}
  missing_required = sorted(required - set(available))
  if missing_required:
    raise RuntimeError("Required services unavailable: %s" % ", ".join(missing_required))
  return available, missing


def main():
  log_dir = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_LOG_DIR
  os.makedirs(log_dir, exist_ok=True)

  cp, torque_bp, torque_v = load_car_params()
  services, missing_services = choose_services()
  sm = messaging.SubMaster(services)

  stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
  fingerprint = str(safe_get(cp, "carFingerprint", "DNGA")).replace("/", "_")
  csv_path = os.path.join(log_dir, "dnga_v25p_%s_%s.csv" % (fingerprint, stamp))
  meta_path = os.path.join(log_dir, "dnga_v25p_%s_%s_meta.txt" % (fingerprint, stamp))

  fields = [
    "wall_time_iso", "mono_time_ns",
    "enabled", "active", "long_control_state",
    "v_ego_mps", "v_ego_kph", "a_ego_mps2", "standstill",
    "cruise_available", "cruise_enabled", "cruise_set_mps", "v_cruise_kph",
    "gas_pressed", "brake_pressed",
    "planner_accel_0", "planner_accel_1", "planner_speed_0", "planner_speed_5",
    "planner_source", "planner_has_lead",
    "cc_accel_request", "cc_steer_request",
    "lead_status", "lead_d_rel_m", "lead_v_rel_mps",
    "vision_turn_state", "vision_turn_speed_mps", "vision_current_lat_accel",
    "turn_speed_state", "turn_speed_mps", "dist_to_turn_m",
    "lateral_curvature_0", "controls_desired_curvature", "model_curvature_est",
    "selected_curvature", "estimated_lat_accel_mps2", "curve_decel_candidate",
    "steering_angle_deg", "steering_rate_deg", "steering_pressed",
    "steering_torque_driver", "steering_torque_eps",
    "left_blinker", "right_blinker", "single_blinker",
    "steer_max_interp", "steer_requested_raw",
    "driver_allowed_min", "driver_allowed_max",
    "tx_steer_applied_raw", "tx_steer_src", "tx_1d0_hex",
    "steer_limited_delta", "steer_limited",
    "tx_271_src", "tx_271_hex", "tx_271_brake_state",
    "tx_271_unknown_byte2", "tx_271_pump_reaction2",
    "tx_271_pump_positive", "tx_271_magnitude_raw", "tx_271_decel_cmd",
    "tx_273_src", "tx_273_hex", "tx_273_set_speed_kph",
    "tx_273_acc_cmd_mps", "tx_273_state_byte4",
    "tx_273_is_accel", "tx_273_is_decel",
  ]

  stop = {"value": False}

  def handle_stop(_sig, _frame):
    stop["value"] = True

  signal.signal(signal.SIGINT, handle_stop)
  signal.signal(signal.SIGTERM, handle_stop)

  with open(meta_path, "w") as meta:
    meta.write("DNGA V2.5P all-in-one logger\n")
    meta.write("Started: %s\n" % datetime.now().isoformat())
    meta.write("Fingerprint: %s\n" % fingerprint)
    meta.write("torqueBP: %r\n" % torque_bp)
    meta.write("torqueV: %r\n" % torque_v)
    meta.write("Services: %r\n" % services)
    meta.write("Missing optional services: %r\n" % missing_services)
    meta.write("CSV: %s\n" % csv_path)

  last_tx = {}
  rows_since_flush = 0
  rk = Ratekeeper(LOG_HZ, print_delay_threshold=None) if Ratekeeper is not None else None
  next_time = time.monotonic()

  with open(csv_path, "w", buffering=1) as csv_file:
    writer = csv.DictWriter(csv_file, fieldnames=fields, extrasaction="ignore")
    writer.writeheader()

    print("Logging to:", csv_path)
    print("Metadata:", meta_path)
    print("Press Ctrl+C to stop.")

    while not stop["value"]:
      sm.update(0)

      if "sendcan" in services and sm.updated.get("sendcan", False):
        last_tx.update(get_latest_tx_frames(sm["sendcan"]))

      cs = sm["carState"]
      cc = sm["carControl"]
      controls = sm["controlsState"]

      lp = sm["longitudinalPlan"] if "longitudinalPlan" in services else None
      latp = sm["lateralPlan"] if "lateralPlan" in services else None
      radar = sm["radarState"] if "radarState" in services else None
      model = sm["modelV2"] if "modelV2" in services else None

      actuators = safe_get(cc, "actuators", None)
      cruise = safe_get(cs, "cruiseState", None)

      v_ego = finite(safe_get(cs, "vEgo", math.nan))
      a_ego = finite(safe_get(cs, "aEgo", math.nan))
      cc_accel = finite(safe_get(actuators, "accel", math.nan))
      cc_steer = finite(safe_get(actuators, "steer", math.nan))

      accels = safe_get(lp, "accels", []) if lp is not None else []
      speeds = safe_get(lp, "speeds", []) if lp is not None else []
      planner_accel_0 = finite(first_item(accels, math.nan, 0))
      planner_accel_1 = finite(first_item(accels, math.nan, 1))
      planner_speed_0 = finite(first_item(speeds, math.nan, 0))
      planner_speed_5 = finite(first_item(speeds, math.nan, 5))

      planner_source_value = first_existing(
        lp,
        ["longitudinalPlanSource", "source", "longitudinalPlanSourceDEPRECATED"],
        "",
      ) if lp is not None else ""
      planner_source = enum_text(planner_source_value)
      planner_has_lead = bool_int(first_existing(lp, ["hasLead", "hasLeadDEPRECATED"], False)) if lp is not None else 0

      lead_one = nested_get(radar, "leadOne", None) if radar is not None else None
      lead_status = bool_int(safe_get(lead_one, "status", False))
      lead_d_rel = finite(safe_get(lead_one, "dRel", math.nan))
      lead_v_rel = finite(safe_get(lead_one, "vRel", math.nan))

      vision_turn_state = enum_text(first_existing(
        lp, ["visionTurnControllerState", "visionTurnState"], ""
      )) if lp is not None else ""
      vision_turn_speed = finite(first_existing(
        lp, ["visionTurnSpeed", "visionTurnControllerSpeed"], math.nan
      )) if lp is not None else math.nan
      vision_lat_accel = finite(first_existing(
        lp, ["visionCurrentLatAcc", "visionCurrentLatAccel"], math.nan
      )) if lp is not None else math.nan
      turn_speed_state = enum_text(first_existing(
        lp, ["turnSpeedControlState", "turnControllerState"], ""
      )) if lp is not None else ""
      turn_speed = finite(first_existing(
        lp, ["turnSpeed", "turnSpeedLimit", "turnSpeedTarget"], math.nan
      )) if lp is not None else math.nan
      dist_to_turn = finite(first_existing(
        lp, ["distToTurn", "turnSpeedLimitEndDistance"], math.nan
      )) if lp is not None else math.nan

      lateral_curvature = finite(first_item(
        safe_get(latp, "curvatures", []) if latp is not None else [],
        math.nan,
        0,
      ))
      controls_curvature = finite(first_existing(
        controls,
        ["desiredCurvature", "curvature"],
        math.nan,
      ))
      model_curvature = estimate_model_curvature(model) if model is not None else math.nan

      curvature_candidates = [
        c for c in (lateral_curvature, controls_curvature, model_curvature)
        if math.isfinite(c)
      ]
      selected_curvature = max(curvature_candidates, key=lambda x: abs(x)) if curvature_candidates else math.nan
      estimated_lat_accel = (
        v_ego * v_ego * abs(selected_curvature)
        if math.isfinite(v_ego) and math.isfinite(selected_curvature)
        else math.nan
      )

      source_lower = planner_source.lower()
      curve_source = ("turn" in source_lower) or ("curve" in source_lower)
      curve_decel_candidate = bool_int(
        (curve_source or (
          math.isfinite(estimated_lat_accel) and estimated_lat_accel >= 0.7
        )) and
        math.isfinite(planner_accel_0) and planner_accel_0 < -0.02 and
        not lead_status
      )

      steer_max = finite(interp(v_ego, torque_bp, torque_v), 1.0)
      steer_max = max(1.0, steer_max)
      requested_raw = round(cc_steer * steer_max) if math.isfinite(cc_steer) else math.nan

      eps_torque = finite(safe_get(cs, "steeringTorqueEps", math.nan))
      single_blinker = bool_int(
        bool(safe_get(cs, "leftBlinker", False)) !=
        bool(safe_get(cs, "rightBlinker", False))
      )
      torque_mult = 10.0 if single_blinker else 1.5
      driver_allowed_max = finite(clip(255.0 + eps_torque * torque_mult, 0.0, 255.0))
      driver_allowed_min = finite(clip(-255.0 - eps_torque * torque_mult, -255.0, 0.0))

      steer_frame = last_tx.get(0x1D0, {})
      steer_dat = steer_frame.get("dat")
      applied_raw = decode_steering_lkas(steer_dat)
      steer_delta = (
        requested_raw - applied_raw
        if math.isfinite(requested_raw) and math.isfinite(applied_raw)
        else math.nan
      )

      row = {
        "wall_time_iso": datetime.now().isoformat(timespec="milliseconds"),
        "mono_time_ns": time.monotonic_ns(),
        "enabled": bool_int(safe_get(controls, "enabled", safe_get(cc, "enabled", False))),
        "active": bool_int(safe_get(controls, "active", safe_get(cc, "active", False))),
        "long_control_state": enum_text(first_existing(controls, ["longControlState"], "")),
        "v_ego_mps": v_ego,
        "v_ego_kph": v_ego * 3.6 if math.isfinite(v_ego) else math.nan,
        "a_ego_mps2": a_ego,
        "standstill": bool_int(safe_get(cs, "standstill", False)),
        "cruise_available": bool_int(safe_get(cruise, "available", False)),
        "cruise_enabled": bool_int(safe_get(cruise, "enabled", False)),
        "cruise_set_mps": finite(safe_get(cruise, "speed", math.nan)),
        "v_cruise_kph": finite(first_existing(controls, ["vCruise", "vCruiseCluster"], math.nan)),
        "gas_pressed": bool_int(safe_get(cs, "gasPressed", False)),
        "brake_pressed": bool_int(safe_get(cs, "brakePressed", False)),
        "planner_accel_0": planner_accel_0,
        "planner_accel_1": planner_accel_1,
        "planner_speed_0": planner_speed_0,
        "planner_speed_5": planner_speed_5,
        "planner_source": planner_source,
        "planner_has_lead": planner_has_lead,
        "cc_accel_request": cc_accel,
        "cc_steer_request": cc_steer,
        "lead_status": lead_status,
        "lead_d_rel_m": lead_d_rel,
        "lead_v_rel_mps": lead_v_rel,
        "vision_turn_state": vision_turn_state,
        "vision_turn_speed_mps": vision_turn_speed,
        "vision_current_lat_accel": vision_lat_accel,
        "turn_speed_state": turn_speed_state,
        "turn_speed_mps": turn_speed,
        "dist_to_turn_m": dist_to_turn,
        "lateral_curvature_0": lateral_curvature,
        "controls_desired_curvature": controls_curvature,
        "model_curvature_est": model_curvature,
        "selected_curvature": selected_curvature,
        "estimated_lat_accel_mps2": estimated_lat_accel,
        "curve_decel_candidate": curve_decel_candidate,
        "steering_angle_deg": finite(safe_get(cs, "steeringAngleDeg", math.nan)),
        "steering_rate_deg": finite(safe_get(cs, "steeringRateDeg", math.nan)),
        "steering_pressed": bool_int(safe_get(cs, "steeringPressed", False)),
        "steering_torque_driver": finite(safe_get(cs, "steeringTorque", math.nan)),
        "steering_torque_eps": eps_torque,
        "left_blinker": bool_int(safe_get(cs, "leftBlinker", False)),
        "right_blinker": bool_int(safe_get(cs, "rightBlinker", False)),
        "single_blinker": single_blinker,
        "steer_max_interp": steer_max,
        "steer_requested_raw": requested_raw,
        "driver_allowed_min": driver_allowed_min,
        "driver_allowed_max": driver_allowed_max,
        "tx_steer_applied_raw": applied_raw,
        "tx_steer_src": steer_frame.get("src", ""),
        "tx_1d0_hex": steer_dat.hex() if steer_dat is not None else "",
        "steer_limited_delta": steer_delta,
        "steer_limited": bool_int(math.isfinite(steer_delta) and abs(steer_delta) >= 1.0),
      }

      frame_271 = last_tx.get(0x271, {})
      frame_273 = last_tx.get(0x273, {})
      row["tx_271_src"] = frame_271.get("src", "")
      row["tx_273_src"] = frame_273.get("src", "")
      row.update(decode_acc_brake(frame_271.get("dat")))
      row.update(decode_acc_cmd(frame_273.get("dat")))

      writer.writerow(row)
      rows_since_flush += 1
      if rows_since_flush >= LOG_HZ:
        csv_file.flush()
        rows_since_flush = 0

      if rk is not None:
        rk.keep_time()
      else:
        next_time += 1.0 / LOG_HZ
        delay = next_time - time.monotonic()
        if delay > 0:
          time.sleep(delay)
        else:
          next_time = time.monotonic()

  print("Stopped. CSV saved to:", csv_path)


if __name__ == "__main__":
  main()
