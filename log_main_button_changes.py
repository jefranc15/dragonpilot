#!/usr/bin/env python3
import cereal.messaging as messaging
import time
import csv
import sys

sm = messaging.SubMaster(["can"], poll="can")

last = {}

w = csv.writer(sys.stdout)
w.writerow([
  "time",
  "bus",
  "addr_hex",
  "addr_dec",
  "old_data",
  "new_data"
])
sys.stdout.flush()

while True:
  sm.update(1000)

  for m in sm["can"]:
    key = (m.src, m.address)
    data = bytes(m.dat).hex()

    if key not in last:
      last[key] = data
      continue

    if last[key] != data:
      w.writerow([
        "%.3f" % time.time(),
        m.src,
        hex(m.address),
        m.address,
        last[key],
        data,
      ])
      sys.stdout.flush()
      last[key] = data
