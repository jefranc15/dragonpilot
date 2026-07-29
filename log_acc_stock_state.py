#!/usr/bin/env python3
import csv
import sys
import time
import cereal.messaging as messaging

# Important stock / OP ACC-LKAS candidates:
# 0x208 = PCM_BUTTONS / cruise buttons
# 0x273 = ACC_CMD_HUD candidate
# 0x271 = ACC_BRAKE candidate
# 0x274 = LKAS_HUD candidate
# plus the low-change IDs from your previous main button log
WATCH_ADDRS = {
  0x208, 0x273, 0x271, 0x274,
  0x2C9, 0x244, 0x51E, 0x101, 0x109
}

def sg(obj, path, default=""):
  try:
    for p in path.split("."):
      obj = getattr(obj, p)
    return obj
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

sm = messaging.SubMaster(["can", "carState", "carControl"], poll="can")

w = csv.writer(sys.stdout)
w.writerow([
  "t", "bus", "addr_hex", "addr_dec", "data",
  "b0", "b1", "b2", "b3", "b4", "b5", "b6", "b7",
  "u16_01", "u16_12", "u16_23", "u16_34", "u16_45",
  "vEgoKph",
  "cruiseAvailable", "cruiseEnabled", "cruiseSpeed", "cruiseSpeedCluster",
  "ccEnabled", "ccLongActive", "hudLeadVisible", "hudSetSpeed"
])
sys.stdout.flush()

while True:
  sm.update(1000)

  cs = sm["carState"]
  cc = sm["carControl"]

  base = [
    sg(cs, "vEgo", 0.0) * 3.6,
    sg(cs, "cruiseState.available", ""),
    sg(cs, "cruiseState.enabled", ""),
    sg(cs, "cruiseState.speed", ""),
    sg(cs, "cruiseState.speedCluster", ""),
    sg(cc, "enabled", ""),
    sg(cc, "longActive", ""),
    sg(cc, "hudControl.leadVisible", ""),
    sg(cc, "hudControl.setSpeed", ""),
  ]

  for m in sm["can"]:
    addr = int(m.address)
    if addr not in WATCH_ADDRS:
      continue

    dat = bytes(m.dat)
    w.writerow([
      "%.3f" % time.time(),
      int(m.src),
      "0x%03X" % addr,
      addr,
      dat.hex(),
      b(dat, 0), b(dat, 1), b(dat, 2), b(dat, 3),
      b(dat, 4), b(dat, 5), b(dat, 6), b(dat, 7),
      u16(dat, 0), u16(dat, 1), u16(dat, 2), u16(dat, 3), u16(dat, 4),
    ] + base)
    sys.stdout.flush()
