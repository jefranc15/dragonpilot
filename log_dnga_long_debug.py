#!/usr/bin/env python3
import csv
import sys
import time

import cereal.messaging as messaging

WATCH_ADDRS = {0x271, 0x273, 0x274, 0x277, 0x520}

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

def dec_271(dat):
  try:
    b = bytes(dat)
    state = b[1]
    pump = b[3]
    pump_signed = pump - 256 if pump > 127 else pump
    mag = int.from_bytes(b[4:6], "big")
    return state, pump, pump_signed, mag
  except Exception:
    return "", "", "", ""

services = ["can", "carState", "controlsState", "longitudinalPlan", "carControl", "radarState"]
sm = messaging.SubMaster(services, poll="can")

w = csv.writer(sys.stdout)
w.writerow([
  "t",
  "bus", "addr", "data",
  "brakeState271", "pumpByte271", "pumpSigned271", "brakeMag271",

  "vEgo", "vEgoKph", "aEgo", "standstill",
  "gas", "gasPressed", "brakePressed",

  "cruiseAvailable", "cruiseEnabled", "cruiseSpeed", "cruiseSpeedCluster",
  "controlsEnabled", "controlsActive", "longControlState", "vCruise",

  "ccEnabled", "ccLongActive", "actAccel", "actGas", "actBrake",
  "hudLeadVisible", "hudSetSpeed",

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
    sg(cc, "actuators.accel", ""),
    sg(cc, "actuators.gas", ""),
    sg(cc, "actuators.brake", ""),
    sg(cc, "hudControl.leadVisible", ""),
    sg(cc, "hudControl.setSpeed", ""),

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
      dat.hex(),
      state,
      pump,
      pump_signed,
      mag,
    ] + base)
    sys.stdout.flush()
