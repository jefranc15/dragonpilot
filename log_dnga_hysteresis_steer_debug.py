#!/usr/bin/env python3
import csv
import sys
import time
from cereal import messaging

SERVICES = [
  'can',
  'carState',
  'carControl',
  'controlsState',
  'longitudinalPlan',
  'radarState',
  'liveParameters',
]

def g(obj, path, default=''):
  try:
    cur = obj
    for part in path.split('.'):
      if not hasattr(cur, part):
        return default
      cur = getattr(cur, part)
    return cur
  except Exception:
    return default

def gi(seq, idx, default=''):
  try:
    return seq[idx]
  except Exception:
    return default

def bhex(dat):
  try:
    return bytes(dat).hex().upper()
  except Exception:
    return ''

def u16be(dat, i):
  try:
    dat = bytes(dat)
    return (dat[i] << 8) | dat[i+1]
  except Exception:
    return ''

def s8(v):
  try:
    v = int(v)
    return v - 256 if v >= 128 else v
  except Exception:
    return ''

def b(dat, i):
  try:
    return bytes(dat)[i]
  except Exception:
    return ''

def can_get(last_can, addr, bus):
  return last_can.get((bus, addr), b'')

def brake_state(dat):
  return b(dat, 1)

def brake_pump_s8(dat):
  return s8(b(dat, 3))

def brake_mag(dat):
  return u16be(dat, 4)

fields = [
  't',
  'updated_can',

  # carState
  'vEgo', 'vEgoKph', 'aEgo',
  'gasPressed', 'brakePressed', 'standstill',
  'cruise_enabled', 'cruise_available', 'cruise_standstill',
  'cruise_speed', 'cruise_speed_kph', 'cruise_speedCluster',
  'steeringAngleDeg', 'steeringRateDeg', 'steeringTorqueEps',
  'steeringPressed',
  'leftBlinker', 'rightBlinker',
  'lkas_latch_carState', 'lkasLatch_carState',

  # carControl actuators
  'actAccel', 'actGas', 'actBrake', 'actSteer',
  'actTorque', 'actCurvature', 'actDesiredCurvature',

  # controlsState / planner
  'controls_vPid', 'controls_aTarget',
  'controls_curvature', 'controls_desiredCurvature',
  'controls_lateralControlState',
  'longPlan_hasLead',
  'planSpeed0', 'planSpeed1', 'planSpeed2',
  'planAccel0', 'planAccel1', 'planAccel2',
  'planSource',

  # radar
  'leadOne_status', 'leadOne_dRel', 'leadOne_yRel',
  'leadOne_vRel', 'leadOne_vLead', 'leadOne_vLeadK',
  'leadOne_aLeadK', 'leadOne_modelProb',
  'leadTwo_status', 'leadTwo_dRel', 'leadTwo_vRel', 'leadTwo_vLead',

  # liveParameters
  'live_valid', 'steerRatioLive', 'stiffnessFactor',
  'angleOffsetDeg', 'angleOffsetAverageDeg',

  # CAN 0x271 brake, outgoing bus 128 and stock/camera bus 2
  'can271_128', 'brakeState271_128', 'pump271_128_s8', 'brakeMag271_128',
  'can271_2', 'brakeState271_2', 'pump271_2_s8', 'brakeMag271_2',

  # important raw CAN
  'can273_128', 'can274_128',
  'can273_2', 'can274_2',
  'can2E4_128', 'can191_128', 'can412_128', 'can2E6_128',
  'can2E4_2', 'can191_2', 'can412_2', 'can2E6_2',
]

writer = csv.DictWriter(sys.stdout, fieldnames=fields, extrasaction='ignore')
writer.writeheader()
sys.stdout.flush()

sm = messaging.SubMaster(
  SERVICES,
  ignore_alive=SERVICES,
  ignore_avg_freq=SERVICES,
)

last_can = {}
t0 = time.monotonic()

while True:
  sm.update(50)

  if sm.updated.get('can', False):
    for c in sm['can']:
      try:
        last_can[(int(c.src), int(c.address))] = bytes(c.dat)
      except Exception:
        pass

  cs = sm['carState']
  cc = sm['carControl']
  ctrl = sm['controlsState']
  lp = sm['longitudinalPlan']
  rs = sm['radarState']
  live = sm['liveParameters']

  c271_128 = can_get(last_can, 0x271, 128)
  c271_2 = can_get(last_can, 0x271, 2)

  row = {
    't': round(time.monotonic() - t0, 3),
    'updated_can': int(sm.updated.get('can', False)),

    'vEgo': g(cs, 'vEgo'),
    'vEgoKph': g(cs, 'vEgo') * 3.6 if g(cs, 'vEgo', '') != '' else '',
    'aEgo': g(cs, 'aEgo'),
    'gasPressed': g(cs, 'gasPressed'),
    'brakePressed': g(cs, 'brakePressed'),
    'standstill': g(cs, 'standstill'),
    'cruise_enabled': g(cs, 'cruiseState.enabled'),
    'cruise_available': g(cs, 'cruiseState.available'),
    'cruise_standstill': g(cs, 'cruiseState.standstill'),
    'cruise_speed': g(cs, 'cruiseState.speed'),
    'cruise_speed_kph': g(cs, 'cruiseState.speed') * 3.6 if g(cs, 'cruiseState.speed', '') != '' else '',
    'cruise_speedCluster': g(cs, 'cruiseState.speedCluster'),
    'steeringAngleDeg': g(cs, 'steeringAngleDeg'),
    'steeringRateDeg': g(cs, 'steeringRateDeg'),
    'steeringTorqueEps': g(cs, 'steeringTorqueEps'),
    'steeringPressed': g(cs, 'steeringPressed'),
    'leftBlinker': g(cs, 'leftBlinker'),
    'rightBlinker': g(cs, 'rightBlinker'),
    'lkas_latch_carState': g(cs, 'lkas_latch'),
    'lkasLatch_carState': g(cs, 'lkasLatch'),

    'actAccel': g(cc, 'actuators.accel'),
    'actGas': g(cc, 'actuators.gas'),
    'actBrake': g(cc, 'actuators.brake'),
    'actSteer': g(cc, 'actuators.steer'),
    'actTorque': g(cc, 'actuators.torque'),
    'actCurvature': g(cc, 'actuators.curvature'),
    'actDesiredCurvature': g(cc, 'actuators.desiredCurvature'),

    'controls_vPid': g(ctrl, 'vPid'),
    'controls_aTarget': g(ctrl, 'aTarget'),
    'controls_curvature': g(ctrl, 'curvature'),
    'controls_desiredCurvature': g(ctrl, 'desiredCurvature'),
    'controls_lateralControlState': g(ctrl, 'lateralControlState'),

    'longPlan_hasLead': g(lp, 'hasLead'),
    'planSpeed0': gi(g(lp, 'speeds', []), 0),
    'planSpeed1': gi(g(lp, 'speeds', []), 1),
    'planSpeed2': gi(g(lp, 'speeds', []), 2),
    'planAccel0': gi(g(lp, 'accels', []), 0),
    'planAccel1': gi(g(lp, 'accels', []), 1),
    'planAccel2': gi(g(lp, 'accels', []), 2),
    'planSource': g(lp, 'longitudinalPlanSource'),

    'leadOne_status': g(rs, 'leadOne.status'),
    'leadOne_dRel': g(rs, 'leadOne.dRel'),
    'leadOne_yRel': g(rs, 'leadOne.yRel'),
    'leadOne_vRel': g(rs, 'leadOne.vRel'),
    'leadOne_vLead': g(rs, 'leadOne.vLead'),
    'leadOne_vLeadK': g(rs, 'leadOne.vLeadK'),
    'leadOne_aLeadK': g(rs, 'leadOne.aLeadK'),
    'leadOne_modelProb': g(rs, 'leadOne.modelProb'),
    'leadTwo_status': g(rs, 'leadTwo.status'),
    'leadTwo_dRel': g(rs, 'leadTwo.dRel'),
    'leadTwo_vRel': g(rs, 'leadTwo.vRel'),
    'leadTwo_vLead': g(rs, 'leadTwo.vLead'),

    'live_valid': g(live, 'valid'),
    'steerRatioLive': g(live, 'steerRatio'),
    'stiffnessFactor': g(live, 'stiffnessFactor'),
    'angleOffsetDeg': g(live, 'angleOffsetDeg'),
    'angleOffsetAverageDeg': g(live, 'angleOffsetAverageDeg'),

    'can271_128': bhex(c271_128),
    'brakeState271_128': brake_state(c271_128),
    'pump271_128_s8': brake_pump_s8(c271_128),
    'brakeMag271_128': brake_mag(c271_128),
    'can271_2': bhex(c271_2),
    'brakeState271_2': brake_state(c271_2),
    'pump271_2_s8': brake_pump_s8(c271_2),
    'brakeMag271_2': brake_mag(c271_2),

    'can273_128': bhex(can_get(last_can, 0x273, 128)),
    'can274_128': bhex(can_get(last_can, 0x274, 128)),
    'can273_2': bhex(can_get(last_can, 0x273, 2)),
    'can274_2': bhex(can_get(last_can, 0x274, 2)),

    'can2E4_128': bhex(can_get(last_can, 0x2E4, 128)),
    'can191_128': bhex(can_get(last_can, 0x191, 128)),
    'can412_128': bhex(can_get(last_can, 0x412, 128)),
    'can2E6_128': bhex(can_get(last_can, 0x2E6, 128)),
    'can2E4_2': bhex(can_get(last_can, 0x2E4, 2)),
    'can191_2': bhex(can_get(last_can, 0x191, 2)),
    'can412_2': bhex(can_get(last_can, 0x412, 2)),
    'can2E6_2': bhex(can_get(last_can, 0x2E6, 2)),
  }

  writer.writerow(row)
  sys.stdout.flush()
