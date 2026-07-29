#!/usr/bin/env python3
import csv
import sys
import time

import cereal.messaging as messaging

# ACC / LKAS / steering command watch list.
# 0x271 = ACC_BRAKE
# 0x273 = ACC_CMD
# 0x274 = LKAS_HUD
# 0x2E4 = common Toyota/DNGA steering command candidate
WATCH_ADDRS = {
  0x271, 0x273, 0x274, 0x277,
  0x208, 0x280, 0x1F0, 0x520,
  0x2E4, 0x191, 0x412, 0x2E6
}

def sg(obj, path, default=""):
  try:
    for p in path.split("."):
      obj = getattr(obj, p)
    return obj
  except Exception:
    return default

def first(seq, default=0.0):
  try:
    return seq[0] if len(seq) else default
  except Exception:
    return default

def b(dat, i):
  try:
    return dat[i]
  except Exception:
    return ""

def u16(dat, i):
  try:
    return int.from_bytes(dat[i:i+2], "big")
  except Exception:
    return ""

def dec_271(dat):
  try:
    state = dat[1]
    pump = dat[3]
    pump_signed = pump - 256 if pump > 127 else pump
    mag = int.from_bytes(dat[4:6], "big")
    return state, pump, pump_signed, mag
  except Exception:
    return "", "", "", ""

services = ["can", "carState", "controlsState", "longitudinalPlan", "carControl", "radarState"]
sm = messaging.SubMaster(services, poll="can")

w = csv.writer(sys.stdout)
w.writerow([
  "t", "bus", "addr", "addrDec", "data",

  "raw_b0", "raw_b1", "raw_b2", "raw_b3", "raw_b4", "raw_b5", "raw_b6", "raw_b7",
  "raw_u16_01", "raw_u16_12", "raw_u16_23", "raw_u16_34", "raw_u16_45",

  "brakeState271", "pumpByte271", "pumpSigned271", "brakeMag271",

  "vEgo", "vEgoKph", "aEgo", "standstill",
  "gas", "gasPressed", "brakePressed",

  "cruiseAvailable", "cruiseEnabled", "cruiseSpeed", "cruiseSpeedCluster",
  "controlsEnabled", "controlsActive", "longControlState", "vCruise",

  "ccEnabled", "ccLongActive", "ccLatActive",
  "actAccel", "actGas", "actBrake", "actSteer",
  "hudLeadVisible", "hudSetSpeed",

  "steeringAngleDeg", "steeringRateDeg", "steeringTorqueEps",
  "steeringPressed", "steerOverride", "lkas_latch",

  "steerRatio", "curvature", "desiredCurvature", "desiredCurvatureRate",

  "planSpeed0", "planAccel0",

  "visionTurnSpeed", "turnSpeed", "turnSpeedControlState", "distToTurn",

  "radarLeadStatus", "radarLeadDRel", "radarLeadVRel", "radarLeadARel",
])

while True:
  sm.update(1000)

  cs = sm["carState"]
  controls = sm["controlsState"]
  lp = sm["longitudinalPlan"]
  cc = sm["carControl"]
  rs = sm["radarState"]

  lead1 = sg(rs, "leadOne", None)

  base = [
    sg(cs, "vEgo", 0.0),
    sg(cs, "vEgo", 0.0) * 3.6,
    sg(cs, "aEgo", 0.0),
    sg(cs, "standstill", ""),

    sg(cs, "gas", ""),
    sg(cs, "gasPressed", ""),
    sg(cs, "brakePressed", ""),

    sg(cs, "cruiseState.available", ""),
    sg(cs, "cruiseState.enabled", ""),
    sg(cs, "cruiseState.speed", ""),
    sg(cs, "cruiseState.speedCluster", ""),

    sg(controls, "enabled", ""),
    sg(controls, "active", ""),
    sg(controls, "longControlState", ""),
    sg(controls, "vCruise", ""),

    sg(cc, "enabled", ""),
    sg(cc, "longActive", ""),
    sg(cc, "latActive", ""),
    sg(cc, "actuators.accel", ""),
    sg(cc, "actuators.gas", ""),
    sg(cc, "actuators.brake", ""),
    sg(cc, "actuators.steer", ""),
    sg(cc, "hudControl.leadVisible", ""),
    sg(cc, "hudControl.setSpeed", ""),

    sg(cs, "steeringAngleDeg", ""),
    sg(cs, "steeringRateDeg", ""),
    sg(cs, "steeringTorqueEps", ""),
    sg(cs, "steeringPressed", ""),
    sg(cs, "steerOverride", ""),
    sg(cs, "lkas_latch", ""),

    sg(controls, "steerRatio", ""),
    sg(controls, "curvature", ""),
    sg(controls, "desiredCurvature", ""),
    sg(controls, "desiredCurvatureRate", ""),

    first(sg(lp, "speeds", []), 0.0),
    first(sg(lp, "accels", []), 0.0),

    sg(lp, "visionTurnSpeed", ""),
    sg(lp, "turnSpeed", ""),
    sg(lp, "turnSpeedControlState", ""),
    sg(lp, "distToTurn", ""),

    sg(lead1, "status", ""),
    sg(lead1, "dRel", ""),
    sg(lead1, "vRel", ""),
    sg(lead1, "aRel", ""),
  ]

  for m in sm["can"]:
    addr = int(m.address)
    bus = int(m.src)

    if addr not in WATCH_ADDRS:
      continue

    dat = bytes(m.dat)
    state, pump, pump_signed, mag = dec_271(dat) if addr == 0x271 else ("", "", "", "")

    w.writerow([
      "%.3f" % time.time(),
      bus,
      "0x%03X" % addr,
      addr,
      dat.hex(),

      b(dat, 0), b(dat, 1), b(dat, 2), b(dat, 3),
      b(dat, 4), b(dat, 5), b(dat, 6), b(dat, 7),
      u16(dat, 0), u16(dat, 1), u16(dat, 2), u16(dat, 3), u16(dat, 4),

      state, pump, pump_signed, mag,
    ] + base)

    sys.stdout.flush()
