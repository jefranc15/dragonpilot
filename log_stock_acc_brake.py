#!/usr/bin/env python3
import time
from cereal import messaging

WATCH = {0x271, 0x273, 0x274, 0x277}

sm = messaging.SubMaster(["can", "carState", "controlsState"])

print("t,bus,addr,data,vEgo,aEgo,gas,gasPressed,brake,brakePressed,cruiseEnabled,controlsEnabled,alert1,alert2", flush=True)

while True:
  sm.update(1000)

  t = time.time()
  cs = sm["carState"]
  ctrl = sm["controlsState"]

  base = "%.3f,%%d,0x%%X,%%s,%.3f,%.3f,%.3f,%s,%.3f,%s,%s,%s,%s,%s" % (
    t,
    cs.vEgo,
    cs.aEgo,
    cs.gas,
    cs.gasPressed,
    cs.brake,
    cs.brakePressed,
    cs.cruiseState.enabled,
    ctrl.enabled,
    (getattr(ctrl, "alertText1", "") or "").replace(",", " "),
    (getattr(ctrl, "alertText2", "") or "").replace(",", " "),
  )

  if sm.updated["can"]:
    for m in sm["can"]:
      if m.address in WATCH:
        print(base % (m.src, m.address, m.dat.hex()), flush=True)
