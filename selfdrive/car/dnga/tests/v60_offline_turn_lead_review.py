#!/usr/bin/env python3
"""Read-only v6 R2 log review. No CAN output or actuator changes.

Inputs: a decoded, time-aligned 20 Hz CSV with the same fields as the
November 12 user's exported rlog analysis. This script is not imported
by the runtime longitudinal controller.
"""
import argparse
import csv
import json
import math
from collections import deque


def number(row, key, default=0.0):
  try:
    x = float(row.get(key, default))
    return x if math.isfinite(x) else default
  except (ValueError, TypeError):
    return default


def truth(row, key, default=False):
  x = row.get(key)
  if x is None or str(x).strip() == "":
    return default
  return str(x).strip().lower() in ("true", "1", "yes")


def speed_kph(row):
  return number(row, "v_kph", number(row, "v") * 3.6)


def target_kph(row):
  return number(row, "vision_target_kph", number(row, "vision_speed") * 3.6)


def turn_advisory(rows, min_speed_kph=54.0, min_error_kph=2.0,
                  max_future_accel=-0.075, sample_window_s=0.35,
                  required_observations=2, rearm_s=0.75):
  """Research-only early-turn advisory, NOT permission to command braking."""
  q = deque()
  events = []
  last_positive = -1e10
  latched = False
  for row in rows:
    t = number(row, "t")
    while q and t - q[0][0] > sample_window_s:
      q.popleft()
    ego = speed_kph(row)
    target = target_kph(row)
    eligible = (
      truth(row, "engaged", True)
      and not truth(row, "driver_brake")
      and not truth(row, "has_lead")
      and row.get("source") == "turn"
      and ego >= min_speed_kph
      and target > 0.0
      and ego - target >= min_error_kph
      and number(row, "p_amin", 1.0) <= max_future_accel
    )
    q.append((t, eligible))
    if eligible:
      last_positive = t
    if latched and t - last_positive > rearm_s:
      latched = False
    if not latched and eligible and sum(v for _, v in q) >= required_observations:
      events.append({
        "time_s": round(t, 3),
        "speed_kph": round(ego, 2),
        "target_kph": round(target, 2),
        "speed_error_kph": round(ego - target, 2),
        "future_accel_mps2": round(number(row, "p_amin"), 3),
        "kind": "READ_ONLY_TURN_ADVISORY"
      })
      latched = True
  return events


def lead_response_timeline(rows):
  """Observational request/response timing; cannot infer regen/friction share."""
  records = list(rows)
  if not records:
    return {}
  predicates = {
    "radar_first_visible": lambda x: truth(x, "lead_status"),
    "future_plan_below_minus_0_05": lambda x: number(x, "p_amin", 1.0) <= -0.05,
    "sent_braking_0x271": lambda x: int(number(x, "brake_state", -1)) == 33,
    "sent_strength_ge_0_20": lambda x: number(x, "brake_norm") >= 0.20,
    "sent_strength_ge_0_45": lambda x: number(x, "brake_norm") >= 0.45 - 1e-6,
    "measured_ae_le_minus_0_5": lambda x: number(x, "a") <= -0.5,
    "driver_brake": lambda x: truth(x, "driver_brake"),
  }
  events = {}
  for name, predicate in predicates.items():
    hit = next((x for x in records if predicate(x)), None)
    if hit is not None:
      events[name] = round(number(hit, "t"), 3)
  first = events.get("sent_braking_0x271")
  if first is not None:
    for name in ("sent_strength_ge_0_20", "sent_strength_ge_0_45",
                 "measured_ae_le_minus_0_5", "driver_brake"):
      if name in events:
        events[name + "_after_first_brake_s"] = round(events[name] - first, 3)
  events["interpretation"] = "Timings do not identify regen/friction share or ECU torque response."
  return events


if __name__ == "__main__":
  p = argparse.ArgumentParser(description=__doc__)
  p.add_argument("csv_file", help="20 Hz time-aligned, decoded rlog CSV")
  p.add_argument("--mode", choices=("turn", "lead"), required=True)
  args = p.parse_args()
  with open(args.csv_file, newline="") as fh:
    data = list(csv.DictReader(fh))
  print(json.dumps(turn_advisory(data) if args.mode == "turn" else
                   lead_response_timeline(data), indent=2))
