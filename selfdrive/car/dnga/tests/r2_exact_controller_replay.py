#!/usr/bin/env python3
"""OFF-CAR same-input replay of V5.0 and the passive R2 instrumented copy.

This never imports CarController, connects to Panda, or transmits CAN.
Requires the device's existing 0.8.13 cereal/runtime and full rlog.bz2 files.
The vehicle's response to hypothetical different commands is NOT simulated.
"""
import argparse
import bisect
import bz2
import csv
import json
import re
import zipfile
from pathlib import Path
from types import SimpleNamespace

from cereal import log

# Exact V5.0 longitudinal.py imports cereal.messaging, whose native
# messaging_pyx is absent in an unbuilt Termux worktree. Replay never uses
# real sockets; both controllers get RecordedFeed below. Bootstrap BEFORE
# importing any longitudinal module. This file is not used by CarController.
from r2_replay_offline_messaging import install_offline_messaging
install_offline_messaging()

from selfdrive.car.dnga.dnga_hybrid_feedback import (
  initialize_hybrid_feedback_state, apply_hybrid_feedback_frame,
)
from selfdrive.car.dnga.longitudinal_kinematic_guard import LongitudinalController as V50
from selfdrive.car.dnga.longitudinal_r2_instrumented import LongitudinalController as R2Probe


class RecordedFeed:
  def __init__(self, topic):
    self.topic = topic
    self.value = None
    self.dirty = False
    self.updated = {topic: False}

  def push(self, value):
    self.value = value
    self.dirty = True

  def update(self, timeout=0):
    self.updated[self.topic] = self.dirty and self.value is not None
    self.dirty = False

  def __getitem__(self, topic):
    assert topic == self.topic
    return self.value


def make_controllers():
  pair = (V50(), R2Probe())
  for controller in pair:
    controller.plan_sm = RecordedFeed('longitudinalPlan')
    controller.radar_sm = RecordedFeed('radarState')
  return pair


def vehicle_state():
  cs = SimpleNamespace(out=None, op_distance_val=1)
  initialize_hybrid_feedback_state(cs)
  return cs


def route_label(path):
  label = path.parent.name if path.name.startswith('rlog') else path.stem
  return label.rsplit('--', 1)[0]


def native_sort(path):
  # Route timestamps contain "--HH-MM-SS"; use only the FINAL segment number.
  # Names like "--14-questionable behavior" must sort as segment 14.
  label = path.parent.name if path.name.startswith('rlog') else path.stem
  route, separator, segment = label.rpartition('--')
  digits = ''
  for char in segment:
    if not char.isdecimal():
      break
    digits += char
  return (route if separator else label, int(digits) if digits else -1)


def iter_rlogs(inputs):
  """Yield one decompressed rlog at a time from paths or a ZIP without extraction."""
  for item in sorted(map(Path, inputs), key=native_sort):
    if item.suffix.lower() == '.zip':
      with zipfile.ZipFile(item) as archive:
        members = sorted(
          [name for name in archive.namelist()
           if name.endswith('/rlog.bz2')
           and re.search(r'\d{4}-\d{2}-\d{2}--\d{2}-\d{2}-\d{2}--\d+',
                         str(Path(name).parent))],
          key=lambda name: native_sort(Path(name))
        )
        if not members:
          raise ValueError('ZIP has no YYYY-MM-DD--HH-MM-SS--N/rlog.bz2 entries: ' + str(item))
        print('ZIP:', item, 'rlog segments:', len(members), flush=True)
        for name in members:
          yield Path(name), bz2.decompress(archive.read(name))
    else:
      yield item, bz2.decompress(item.read_bytes())


def canonical(command):
  if command is None:
    return None
  return (bool(command.enabled), bool(command.lead),
          round(float(command.speed), 6), bool(command.is_accel),
          bool(command.is_decel), int(command.brake_state),
          round(float(command.pump), 6), int(command.magnitude))


def replay(paths, output):
  output = Path(output)
  output.mkdir(parents=True, exist_ok=True)
  route = None
  frame = equal = different = 0
  events, commands, sent_brakes, samples, panda_states = [], [], [], [], []
  cs = vehicle_state()
  pair = make_controllers()

  for path, raw in iter_rlogs(paths):
    if route_label(path) != route:
      route = route_label(path)
      pair = make_controllers()
      cs = vehicle_state()
      frame = 0
      events.append(dict(event='route_start', route=route))

    # Read exactly one rlog at a time to bound memory on OnePlus 3T.
    for e in log.Event.read_multiple_bytes(raw):
      tag = e.which()
      t_ns = int(e.logMonoTime)
      if tag == 'longitudinalPlan':
        for controller in pair:
          controller.plan_sm.push(e.longitudinalPlan)
      elif tag == 'radarState':
        for controller in pair:
          controller.radar_sm.push(e.radarState)
      elif tag == 'carState':
        cs.out = e.carState
        cs.op_distance_val = max(0, min(2, int(cs.out.distanceLines) - 1))
      elif tag == 'pandaStates':
        for panda in e.pandaStates:
          panda_states.append(dict(route=route, t_ns=t_ns,
            safety_model=str(panda.safetyModel), safety_param=int(panda.safetyParam),
            panda_type=str(panda.pandaType), harness_status=str(panda.harnessStatus),
            controls_allowed=bool(panda.controlsAllowed),
            blocked_count=int(panda.blockedCnt), heartbeat_lost=bool(panda.heartbeatLost)))
      elif tag == 'can':
        for msg in e.can:
          if int(msg.src) == 1:
            apply_hybrid_feedback_frame(cs, frame, int(msg.address), msg.dat)
      elif tag == 'sendcan':
        for msg in e.sendcan:
          if int(msg.address) == 0x271 and int(msg.src) == 0 and len(msg.dat) >= 2:
            sent_brakes.append((route, t_ns, int(msg.dat[1])))
      elif tag == 'carControl' and cs.out is not None:
        cc = e.carControl
        args = (bool(cc.enabled), cs, frame, float(cc.actuators.accel),
                bool(cc.cruiseControl.cancel), bool(cc.hudControl.leadVisible))
        pair[1].r2_replay_log_ns = t_ns
        a = pair[0].update(*args)
        b = pair[1].update(*args)
        if canonical(a) != canonical(b):
          different += 1
        elif a is not None:
          equal += 1
        if a is not None:
          commands.append(dict(route=route, t_ns=t_ns, frame=frame,
                               brake_state=int(a.brake_state),
                               v50=str(canonical(a)), r2_probe=str(canonical(b)),
                               identical=(canonical(a) == canonical(b))))
        frame += 1

    events.extend(dict(route=route, **v) for v in pair[1].r2_events)
    pair[1].r2_events.clear()
    print('Processed:', path, flush=True)
    del raw

  with (output/'r2_internal_events.jsonl').open('w') as f:
    for event in events:
      f.write(json.dumps(event, sort_keys=True) + '\n')
  with (output/'r2_command_comparison.csv').open('w', newline='') as f:
    writer = csv.DictWriter(f, fieldnames=['route', 't_ns', 'frame', 'brake_state',
                                           'v50', 'r2_probe', 'identical'])
    writer.writeheader()
    writer.writerows(commands)

  indices = {}
  for r, t, state in sent_brakes:
    indices.setdefault(r, []).append((t, state))
  for k in indices:
    indices[k].sort()
    seq = indices[k]
    indices[k] = ([e[0] for e in seq], seq)

  total = matched = 0
  for c in commands:
    entry = indices.get(c['route'])
    if not entry:
      continue
    times, seq = entry
    i = bisect.bisect_left(times, c['t_ns'])
    near = [seq[j] for j in (i - 1, i, i + 1) if 0 <= j < len(seq)]
    if not near:
      continue
    nearest = min(near, key=lambda v: abs(v[0] - c['t_ns']))
    if abs(nearest[0] - c['t_ns']) <= 150000000:
      total += 1
      matched += int(nearest[1] == c['brake_state'])
  agreement = matched / total if total else None
  valid = different == 0 and total >= 100 and agreement is not None and agreement >= .95
  summary = dict(v50_r2_equal=equal, v50_r2_different=different,
                 recorded_brake_states_compared=total,
                 recorded_brake_states_matching=matched,
                 recorded_state_agreement=agreement,
                 baseline_replay_valid=valid,
                 physical_vehicle_response_simulated=False)
  (output/'r2_replay_summary.json').write_text(json.dumps(summary, indent=2)+'\n')
  print(json.dumps(summary, indent=2))
  if not valid:
    raise RuntimeError('Baseline reproduction insufficient; R2 internal-state conclusions unverified')


if __name__ == '__main__':
  ap = argparse.ArgumentParser()
  ap.add_argument('rlogs', nargs='+')
  ap.add_argument('--output', default='/data/v50_r2_replay_results')
  args = ap.parse_args()
  replay(args.rlogs, args.output)
, segment) if separator else None
  return (route if separator else label, int(m.group(1)) if m else -1)


def iter_rlogs(inputs):
  """Yield one decompressed rlog at a time from paths or a ZIP without extraction."""
  for item in sorted(map(Path, inputs), key=native_sort):
    if item.suffix.lower() == '.zip':
      with zipfile.ZipFile(item) as archive:
        members = sorted(
          [name for name in archive.namelist()
           if name.endswith('/rlog.bz2')
           and re.search(r'\d{4}-\d{2}-\d{2}--\d{2}-\d{2}-\d{2}--\d+',
                         str(Path(name).parent))],
          key=lambda name: native_sort(Path(name))
        )
        if not members:
          raise ValueError('ZIP has no YYYY-MM-DD--HH-MM-SS--N/rlog.bz2 entries: ' + str(item))
        print('ZIP:', item, 'rlog segments:', len(members), flush=True)
        for name in members:
          yield Path(name), bz2.decompress(archive.read(name))
    else:
      yield item, bz2.decompress(item.read_bytes())


def canonical(command):
  if command is None:
    return None
  return (bool(command.enabled), bool(command.lead),
          round(float(command.speed), 6), bool(command.is_accel),
          bool(command.is_decel), int(command.brake_state),
          round(float(command.pump), 6), int(command.magnitude))


def replay(paths, output):
  output = Path(output)
  output.mkdir(parents=True, exist_ok=True)
  route = None
  frame = equal = different = 0
  events, commands, sent_brakes = [], [], []
  cs = vehicle_state()
  pair = make_controllers()

  for path, raw in iter_rlogs(paths):
    if route_label(path) != route:
      route = route_label(path)
      pair = make_controllers()
      cs = vehicle_state()
      frame = 0
      events.append(dict(event='route_start', route=route))

    # Read exactly one rlog at a time to bound memory on OnePlus 3T.
    for e in log.Event.read_multiple_bytes(raw):
      tag = e.which()
      t_ns = int(e.logMonoTime)
      if tag == 'longitudinalPlan':
        for controller in pair:
          controller.plan_sm.push(e.longitudinalPlan)
      elif tag == 'radarState':
        for controller in pair:
          controller.radar_sm.push(e.radarState)
      elif tag == 'carState':
        cs.out = e.carState
        cs.op_distance_val = max(0, min(2, int(cs.out.distanceLines) - 1))
      elif tag == 'can':
        for msg in e.can:
          if int(msg.src) == 1:
            apply_hybrid_feedback_frame(cs, frame, int(msg.address), msg.dat)
      elif tag == 'sendcan':
        for msg in e.sendcan:
          if int(msg.address) == 0x271 and int(msg.src) == 0 and len(msg.dat) >= 2:
            sent_brakes.append((route, t_ns, int(msg.dat[1])))
      elif tag == 'carControl' and cs.out is not None:
        cc = e.carControl
        args = (bool(cc.enabled), cs, frame, float(cc.actuators.accel),
                bool(cc.cruiseControl.cancel), bool(cc.hudControl.leadVisible))
        a = pair[0].update(*args)
        b = pair[1].update(*args)
        if canonical(a) != canonical(b):
          different += 1
        elif a is not None:
          equal += 1
        if a is not None:
          commands.append(dict(route=route, t_ns=t_ns, frame=frame,
                               brake_state=int(a.brake_state),
                               v50=str(canonical(a)), r2_probe=str(canonical(b)),
                               identical=(canonical(a) == canonical(b))))
        frame += 1

    events.extend(dict(route=route, **v) for v in pair[1].r2_events)
    pair[1].r2_events.clear()
    print('Processed:', path, flush=True)
    del raw

  with (output/'r2_internal_events.jsonl').open('w') as f:
    for event in events:
      f.write(json.dumps(event, sort_keys=True) + '\n')
  with (output/'r2_command_comparison.csv').open('w', newline='') as f:
    writer = csv.DictWriter(f, fieldnames=['route', 't_ns', 'frame', 'brake_state',
                                           'v50', 'r2_probe', 'identical'])
    writer.writeheader()
    writer.writerows(commands)

  indices = {}
  for r, t, state in sent_brakes:
    indices.setdefault(r, []).append((t, state))
  for k in indices:
    indices[k].sort()
    seq = indices[k]
    indices[k] = ([e[0] for e in seq], seq)

  total = matched = 0
  for c in commands:
    entry = indices.get(c['route'])
    if not entry:
      continue
    times, seq = entry
    i = bisect.bisect_left(times, c['t_ns'])
    near = [seq[j] for j in (i - 1, i, i + 1) if 0 <= j < len(seq)]
    if not near:
      continue
    nearest = min(near, key=lambda v: abs(v[0] - c['t_ns']))
    if abs(nearest[0] - c['t_ns']) <= 150000000:
      total += 1
      matched += int(nearest[1] == c['brake_state'])
  agreement = matched / total if total else None
  valid = different == 0 and total >= 100 and agreement is not None and agreement >= .95
  summary = dict(v50_r2_equal=equal, v50_r2_different=different,
                 recorded_brake_states_compared=total,
                 recorded_brake_states_matching=matched,
                 recorded_state_agreement=agreement,
                 baseline_replay_valid=valid,
                 physical_vehicle_response_simulated=False)
  (output/'r2_replay_summary.json').write_text(json.dumps(summary, indent=2)+'\n')
  print(json.dumps(summary, indent=2))
  if not valid:
    raise RuntimeError('Baseline reproduction insufficient; R2 internal-state conclusions unverified')


if __name__ == '__main__':
  ap = argparse.ArgumentParser()
  ap.add_argument('rlogs', nargs='+')
  ap.add_argument('--output', default='/data/v50_r2_replay_results')
  args = ap.parse_args()
  replay(args.rlogs, args.output)
