#!/usr/bin/env python3
import time
from cereal import messaging

WATCH = {0x271, 0x273, 0x274, 0x277}

sm = messaging.SubMaster(["can", "carState", "controlsState"])

print(
  "t,bus,addr,data,"
  "vEgo_ms,vEgo_kph,aEgo,"
  "gas,gasPressed,brake,brakePressed,"
  "cruiseEnabled,controlsEnabled,"
  "brakeState,pumpByte,brakeMag,"
  "alert1,alert2",
  flush=True
)

while True:
  sm.update(1000)

  t = time.time()
  cs = sm["carState"]
  ctrl = sm["controlsState"]

  v_kph = cs.vEgo * 3.6

  if sm.updated["can"]:
    for m in sm["can"]:
      if m.address not in WATCH:
        continue

      dat = bytes(m.dat)

      brake_state = ""
      pump_byte = ""
      brake_mag = ""

      # Decode ACC_BRAKE 0x271 pattern:
      # byte 1 = state, byte 3 = pump/reaction, bytes 4-5 = magnitude
      if m.address == 0x271 and len(dat) >= 6:
        brake_state = "0x%02X" % dat[1]
        pump_byte = "0x%02X" % dat[3]
        brake_mag = int.from_bytes(dat[4:6], "big")

      print(
        "%.3f,%d,0x%X,%s,"
        "%.3f,%.1f,%.3f,"
        "%.3f,%s,%.3f,%s,"
        "%s,%s,"
        "%s,%s,%s,"
        "%s,%s" % (
          t,
          m.src,
          m.address,
          dat.hex(),

          cs.vEgo,
          v_kph,
          cs.aEgo,

          cs.gas,
          cs.gasPressed,
          cs.brake,
          cs.brakePressed,

          cs.cruiseState.enabled,
          ctrl.enabled,

          brake_state,
          pump_byte,
          brake_mag,

          (getattr(ctrl, "alertText1", "") or "").replace(",", " "),
          (getattr(ctrl, "alertText2", "") or "").replace(",", " "),
        ),
        flush=True
      )
