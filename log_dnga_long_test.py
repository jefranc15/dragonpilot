#!/usr/bin/env python3
import time
from cereal import messaging

WATCH = {0x271, 0x273, 0x274, 0x277}

sm = messaging.SubMaster(["can", "carState", "controlsState", "longitudinalPlan", "carControl"])

print("t,bus,addr,data,vEgo,aEgo,gas,gasPressed,brake,brakePressed,cruiseEnabled,controlsEnabled,controlsActive,actAccel,actGas,actBrake,actSteer,planSpeed0,planAccel0,alert1,alert2", flush=True)

while True:
  sm.update(1000)

  t = time.time()
  cs = sm["carState"]
  ctrl = sm["controlsState"]
  lp = sm["longitudinalPlan"]
  cc = sm["carControl"]

  act = cc.actuators

  try:
    plan_speed0 = list(lp.speeds)[0] if len(lp.speeds) else 0.0
  except Exception:
    plan_speed0 = 0.0

  try:
    plan_accel0 = list(lp.accels)[0] if len(lp.accels) else 0.0
  except Exception:
    plan_accel0 = 0.0

  base = "%.3f,%%d,0x%%X,%%s,%.3f,%.3f,%.3f,%s,%.3f,%s,%s,%s,%s,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%s,%s" % (
    t,
    cs.vEgo,
    cs.aEgo,
    cs.gas,
    cs.gasPressed,
    cs.brake,
    cs.brakePressed,
    cs.cruiseState.enabled,
    ctrl.enabled,
    getattr(ctrl, "active", False),
    getattr(act, "accel", 0.0),
    getattr(act, "gas", 0.0),
    getattr(act, "brake", 0.0),
    getattr(act, "steer", 0.0),
    plan_speed0,
    plan_accel0,
    (getattr(ctrl, "alertText1", "") or "").replace(",", " "),
    (getattr(ctrl, "alertText2", "") or "").replace(",", " "),
  )

  if sm.updated["can"]:
    for m in sm["can"]:
      if m.address in WATCH:
        print(base % (m.src, m.address, m.dat.hex()), flush=True)
