#!/usr/bin/env python3
"""Read-only DNGA V5.0 OP brake response audit of time-aligned paired CAN CSV.

Does NOT write CAN, modify actuators, or establish calibrated brake pressure.
The paired CSV format is the output of the Nov 5 offline CAN decoder.
"""
import argparse
import csv
import json
import math
from pathlib import Path
from statistics import median

def boolean(value):
  return str(value).strip().lower() in ("true", "1", "yes")

def load_csv(path):
  out = []
  with open(path, newline="") as stream:
    for row in csv.DictReader(stream):
      try:
        t, v, a, rpm = (float(row[k]) for k in ("t", "v", "a", "rpm"))
        friction = float(row["fric"])
        raw_mag = int(float(row["brk_raw"]))
        if not all(math.isfinite(x) for x in (t, v, a, rpm, friction)):
          continue
        out.append(dict(t=t, v=v, a=a, rpm=rpm, friction=friction,
                        brake=max(0.0, (200-(raw_mag & 0xff))/100.0),
                        op=boolean(row["op"]), oem=boolean(row["oem"])))
      except (ValueError, KeyError, TypeError):
        continue
  return sorted(out, key=lambda row: row["t"])

def episodes(samples, max_gap_s=0.18):
  result, current = [], []
  for row in samples:
    if row["op"] and current and 0 <= row["t"]-current[-1]["t"] <= max_gap_s:
      current.append(row)
    else:
      if current:
        result.append(current)
      current = [row] if row["op"] else []
  if current:
    result.append(current)
  return result

def sustained_decel_s(rows, threshold=-0.15):
  for i in range(len(rows)-1):
    if rows[i]["a"] <= threshold and rows[i+1]["a"] <= threshold:
      return round(rows[i]["t"]-rows[0]["t"], 3)
  return None

def summarize(rows, source):
  a, b = rows[0], rows[-1]
  duration=b["t"]-a["t"]
  speed_change=(b["v"]-a["v"])*3.6
  gain=max(r["rpm"] for r in rows)-a["rpm"]
  med=median(r["a"] for r in rows)
  friction_max=max(r["friction"] for r in rows)
  return dict(
    source=source, start_t=round(a["t"],3), end_t=round(b["t"],3),
    duration_s=round(duration,3), count=len(rows),
    start_kph=round(a["v"]*3.6,2), delta_kph=round(speed_change,2),
    brake_start=round(a["brake"],3), brake_end=round(b["brake"],3),
    brake_max=round(max(r["brake"] for r in rows),3),
    friction_max=round(friction_max,1), rpm_start=int(a["rpm"]),
    rpm_max=int(max(r["rpm"] for r in rows)), rpm_gain=int(gain),
    median_aEgo=round(med,3), decel_onset_s=sustained_decel_s(rows),
    oem_braking_fraction=round(sum(r["oem"] for r in rows)/len(rows),3),
    weak_response_candidate=(duration>=1.4 and speed_change>-1.0 and med>-.1),
    rev_candidate=(duration>=.8 and gain>=700 and friction_max>=10))

def write_csv(path, rows):
  if not rows:
    return
  with path.open("w", newline="") as stream:
    w=csv.DictWriter(stream,fieldnames=list(rows[0]))
    w.writeheader()
    w.writerows(rows)

def main():
  p=argparse.ArgumentParser(description=__doc__)
  p.add_argument("paired_csv",nargs="+")
  p.add_argument("--output",default="v50_long_audit")
  args=p.parse_args()
  all_events, reentries=[],[]
  for filename in args.paired_csv:
    items=[summarize(e,Path(filename).name) for e in episodes(load_csv(filename))]
    for old,new in zip(items, items[1:]):
      gap=new["start_t"]-old["end_t"]
      if 0 < gap <= .8:
        reentries.append(dict(source=Path(filename).name,
                              prior_end_t=old["end_t"], next_start_t=new["start_t"],
                              gap_s=round(gap,3)))
    all_events.extend(items)
  destination=Path(args.output)
  destination.mkdir(parents=True, exist_ok=True)
  write_csv(destination/"braking_episodes.csv",all_events)
  write_csv(destination/"short_reentries.csv",reentries)
  counts=dict(sampled_brake_episodes=len(all_events),
              weak_response_candidates=sum(e["weak_response_candidate"] for e in all_events),
              rpm_rise_candidates=sum(e["rev_candidate"] for e in all_events),
              short_reentries=len(reentries))
  (destination/"summary.json").write_text(json.dumps(dict(
    counts=counts, weak_cases=[e for e in all_events if e["weak_response_candidate"]],
    rpm_cases=[e for e in all_events if e["rev_candidate"]],
    limitations=["Sampled CAN pairing can merge/miss brief native 0x271 episodes",
                 "0x08C is not calibrated brake force",
                 "0x275 follows OEM camera demand, not OP brake pressure",
                 "Lead/planner context and physical causality are not established",
                 "Diagnostic flags are NOT authorization to alter live actuation"]), indent=2))
  print(json.dumps(counts,indent=2))

if __name__=="__main__":
  main()
