#!/usr/bin/env python3
"""OFF-CAR R2 surge screen over r2_20hz_state_trace.csv. No CAN/vehicle imports.

IMPORTANT: V5.0 'lead' input derives from HUD leadVisible, which can be
true with no radar lead. Use plan_has_lead to screen and separately confirm
recorded radar.leadOne.status before asserting a no-lead event.
"""
import argparse
import csv
import json
from pathlib import Path


def yes(value):
  return str(value).strip().lower() in ('1', 'true', 'yes')


def surge_candidate(row):
  try:
    return (int(row['brake_state']) == 1
            and yes(row['enabled'])
            and not yes(row['plan_has_lead'])
            and not yes(row['pedal_gas'])
            and not yes(row['pedal_brake'])
            and not yes(row['is_accel'])
            and float(row['a_ego']) > 0.5
            and float(row['pid_accel']) < -0.5)
  except (KeyError, ValueError, TypeError):
    return False


def find_episodes(files):
  episodes = []
  for filename in map(Path, files):
    with filename.open(newline='') as stream:
      current = []

      def finish():
        nonlocal current
        if current:
          episodes.append(dict(
            file=str(filename), route=current[0]['route'],
            begin_ns=int(current[0]['t_ns']),
            end_ns=int(current[-1]['t_ns']),
            duration_s=round((int(current[-1]['t_ns']) - int(current[0]['t_ns']))/1e9, 3),
            n=len(current),
            max_a_ego=round(max(float(r['a_ego']) for r in current), 3),
            min_pid_accel=round(min(float(r['pid_accel']) for r in current), 3),
            max_speed_kph=round(max(float(r['v_ego']) for r in current)*3.6, 2),
            plan_sources=sorted({r['plan_source'] for r in current}),
          ))
        current = []

      for row in csv.DictReader(stream):
        if current and (row['route'] != current[-1]['route']
                        or int(row['t_ns']) - int(current[-1]['t_ns']) > 150000000):
          finish()
        if surge_candidate(row):
          current.append(row)
        else:
          finish()
      finish()
  return episodes


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('traces', nargs='+')
  parser.add_argument('--json', help='Write JSON episodes')
  options = parser.parse_args()
  results = find_episodes(options.traces)
  for event in results:
    print(json.dumps(event, sort_keys=True))
  print('Episodes:', len(results), 'samples:', sum(e['n'] for e in results))
  if options.json:
    Path(options.json).write_text(json.dumps(results, indent=2) + '\n')
