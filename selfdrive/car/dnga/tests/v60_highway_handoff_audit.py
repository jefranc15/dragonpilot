#!/usr/bin/env python3
"""Read-only DNGA HEV post-brake release audit from decoded, aligned CSV.

NO CAN output, no connection to the vehicle, no actuator settings.
Camera bus-2 intent does NOT prove it reaches the powertrain.
"""
import argparse
import csv
import json
import math


def num(r, k, default=0.):
  try:
    v = float(r.get(k, default))
    return v if math.isfinite(v) else default
  except (ValueError, TypeError):
    return default


def bit(r, k):
  x = r.get(k, False)
  return x is True or str(x).strip().lower() in ("true", "1", "yes")


def runs(rows, predicate, duration=0.25, max_gap=0.12):
  events, current = [], []
  for r in rows:
    t = num(r, "t")
    if predicate(r):
      if current and t - num(current[-1], "t") > max_gap:
        if num(current[-1], "t") - num(current[0], "t") >= duration:
          events.append(current)
        current = []
      current.append(r)
    elif current:
      if num(current[-1], "t") - num(current[0], "t") >= duration:
        events.append(current)
      current = []
  if current and num(current[-1], "t") - num(current[0], "t") >= duration:
    events.append(current)
  return events


def anomalies(rows):
  predicates = {
    "NEUTRAL_POSITIVE_ACCEL_NEGATIVE_PID": lambda r: (
      bit(r,"enabled_y") and num(r,"pedal")<=100 and not bit(r,"driver_brake")
      and num(r,"brake_state")==1 and not bit(r,"is_accel")
      and not bit(r,"is_decel") and num(r,"a")>=0.8 and num(r,"accel")<=-0.5
    ),
    "NEUTRAL_DECEL_POSITIVE_PID": lambda r: (
      bit(r,"enabled_y") and num(r,"pedal")<=100
      and num(r,"brake_state")==1 and not bit(r,"is_accel")
      and not bit(r,"is_decel") and num(r,"a")<=-0.4
      and num(r,"rpm")>=2500 and num(r,"accel")>=0.5
    ),
    "BRAKING_POSITIVE_PID_HIGH_RPM": lambda r: (
      bit(r,"enabled_y") and num(r,"pedal")<=100
      and num(r,"brake_state")==33 and num(r,"rpm")>=2500
      and num(r,"accel")>=0.5 and num(r,"p_a0")<=0
    ),
    "CAMERA_INTENT_OP_DECEL_CONFLICT": lambda r: (
      bit(r,"camera_enabled") and bit(r,"camera_is_accel")
      and bit(r,"is_decel") and not bit(r,"is_accel")
    )
  }
  found = []
  for kind, pred in predicates.items():
    min_duration = 0.75 if kind == "BRAKING_POSITIVE_PID_HIGH_RPM" else 0.25
    for group in runs(rows, pred, min_duration):
      found.append({
        "kind":kind,
        "start_t":round(num(group[0],"t"),3),
        "end_t":round(num(group[-1],"t"),3),
        "duration_s":round(num(group[-1],"t")-num(group[0],"t"),2),
        "peak_rpm":round(max(num(x,"rpm") for x in group)),
        "peak_aego":round(max(num(x,"a") for x in group),3)
      })
  return sorted(found, key=lambda x:(x["start_t"],x["kind"]))


def ready_releases(rows, horizon_s=4.0):
  results=[]
  for i in range(1,len(rows)):
    prev, row=rows[i-1],rows[i]
    if num(prev,"brake_state")!=33 or num(row,"brake_state")!=1:
      continue
    t=num(row,"t")
    after=[x for x in rows[i:] if 0<=num(x,"t")-t<=horizon_s]
    if len(after)<8:
      continue
    no_pedal=[x for x in after if num(x,"pedal")<=100 and not bit(x,"driver_brake")]
    if not no_pedal:
      continue
    results.append({
      "release_t":round(t,3),
      "speed_kph":round(num(row,"v_kph"),2),
      "set_speed_kph":round(num(row,"set_speed"),2),
      "peak_aego_next_4s":round(max(num(x,"a") for x in no_pedal),3),
      "peak_rpm_next_4s":round(max(num(x,"rpm") for x in no_pedal)),
      "driver_pedal_occurred":any(num(x,"pedal")>100 for x in after)
    })
  return results


def audit(rows):
  return {"samples":len(rows),"ready_releases":ready_releases(rows),
          "anomalies":anomalies(rows),
          "limitations":"READ ONLY. Camera bus-2 intent does not prove Panda forwarding/ECU acceptance. Raw 0x275 does not measure regen/friction split."}


if __name__=="__main__":
  parser=argparse.ArgumentParser(description=__doc__)
  parser.add_argument("aligned_csv",help="Decoded rlog time-aligned CSV, not live CAN")
  args=parser.parse_args()
  with open(args.aligned_csv,newline="") as f:
    rows=sorted(csv.DictReader(f),key=lambda r:num(r,"t"))
  print(json.dumps(audit(rows),indent=2))
