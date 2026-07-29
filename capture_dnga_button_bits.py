#!/usr/bin/env python3
import csv
import sys
import time
import cereal.messaging as messaging

# 0x208 = decimal 520, your PCM_BUTTONS frame
# 0x280 / 0x1F0 = distance / UI / button candidates from your previous logs
# 0x520 = real hex 0x520, included just in case
WATCH_ADDRS = {0x208, 0x280, 0x1F0, 0x520}

def bits_set(dat):
  out = []
  for i, byte in enumerate(dat):
    for bit in range(8):
      if byte & (1 << bit):
        out.append(f"b{i}.{bit}")
  return " ".join(out)

def diff_bits(prev, dat):
  out = []
  max_len = max(len(prev), len(dat), 8)
  prev = prev.ljust(max_len, b"\x00")
  dat = dat.ljust(max_len, b"\x00")

  for i in range(max_len):
    changed = prev[i] ^ dat[i]
    if not changed:
      continue
    for bit in range(8):
      if changed & (1 << bit):
        old = 1 if prev[i] & (1 << bit) else 0
        new = 1 if dat[i] & (1 << bit) else 0
        out.append(f"b{i}.{bit}:{old}->{new}")
  return " ".join(out)

def bin_bytes(dat):
  return " ".join(f"{x:08b}" for x in dat)

def dec_bytes(dat):
  return " ".join(str(x) for x in dat)

sm = messaging.SubMaster(["can"], poll="can")

last = {}
t0 = time.monotonic()

w = csv.writer(sys.stdout)
w.writerow([
  "t", "rel_t", "bus", "addr_hex", "addr_dec",
  "data_hex", "bytes_dec", "bytes_bin",
  "event", "changed_bits_lsb", "set_bits_lsb"
])
sys.stdout.flush()

while True:
  sm.update(1000)

  for m in sm["can"]:
    addr = int(m.address)
    bus = int(m.src)

    if addr not in WATCH_ADDRS:
      continue

    dat = bytes(m.dat)
    key = (bus, addr)
    prev = last.get(key)

    if prev is None:
      event = "FIRST"
      changed = ""
    elif prev != dat:
      event = "CHANGE"
      changed = diff_bits(prev, dat)
    else:
      continue

    last[key] = dat

    w.writerow([
      "%.6f" % time.time(),
      "%.3f" % (time.monotonic() - t0),
      bus,
      "0x%03X" % addr,
      addr,
      dat.hex(),
      dec_bytes(dat),
      bin_bytes(dat),
      event,
      changed,
      bits_set(dat),
    ])
    sys.stdout.flush()
