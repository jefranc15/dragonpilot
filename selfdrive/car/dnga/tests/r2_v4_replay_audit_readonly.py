#!/usr/bin/env python3
"""Read-only audit of an R2/V4 OFF-CAR replay CSV.

This utility never imports the live controller, packs CAN messages, or
changes OpenPilot/Panda. Use it with the offline V4 replay results.
"""
import argparse
import csv
import json
from collections import Counter

def is_true(value):
  return str(value).strip().lower() in ('true', '1', 'yes')

def audit(path):
  totals = Counter()
  with open(path, newline='') as inp:
    for row in csv.DictReader(inp):
      totals['frames'] += 1
      original = int(row['original_state'])
      proposed = int(row['v4_state'])
      if proposed == 0x21:
        totals['proposed_braking'] += 1
        if original == 0x01:
          totals['new_braking_relative_to_v50_ready'] += 1
      if original == 0x21 and proposed == 0x01:
        totals['missing_braking_relative_to_v50'] += 1
      if is_true(row.get('bridge')):
        totals['source_bridge_frames'] += 1
      if is_true(row.get('unserved_negative_demand')):
        totals['unserved_negative_demand'] += 1
      if is_true(row.get('calibration_outside_stock')):
        totals['outside_stock_calibration'] += 1
      if is_true(row.get('feedback_stale')):
        totals['stale_feedback_frames'] += 1
      if is_true(row.get('v4_is_accel')):
        totals['proposed_accel'] += 1
        try:
          brake_raw = float(row.get('hybrid_brake_raw') or 0)
          friction = float(row.get('friction_raw') or 0)
        except ValueError:
          totals['invalid_feedback_on_accel'] += 1
        else:
          if brake_raw < 0 or friction > 0:
            totals['accel_with_recorded_brake_feedback'] += 1
  return dict(totals)

if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('replay_csv')
  args = parser.parse_args()
  print(json.dumps(audit(args.replay_csv), sort_keys=True, indent=2))
