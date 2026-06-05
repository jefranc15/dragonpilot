#!/usr/bin/env python3
import math
import re
import subprocess
import time

from cereal import messaging

PATTERNS = [
  # Android/NEOS format: gps Location[ 14.998046,120.783487 acc=...]
  re.compile(r'(?P<provider>\w+)\s+Location\[\s*(?P<lat>-?\d+(?:\.\d+)?),(?P<lon>-?\d+(?:\.\d+)?)(?P<rest>[^\]]*)\]'),

  # Other Android format: Location[gps 14.998046,120.783487 ...]
  re.compile(r'Location\[(?P<provider>[^\s]+)\s+(?P<lat>-?\d+(?:\.\d+)?),(?P<lon>-?\d+(?:\.\d+)?)(?P<rest>[^\]]*)\]'),
]

def find_num(rest, names, default):
  for name in names:
    m = re.search(r'\b' + re.escape(name) + r'=(-?\d+(?:\.\d+)?)', rest)
    if m:
      try:
        return float(m.group(1))
      except Exception:
        pass
  return default

def get_android_location():
  try:
    out = subprocess.check_output(["dumpsys", "location"], text=True, errors="ignore", timeout=2)
  except Exception:
    return None

  best = None

  for pat in PATTERNS:
    for m in pat.finditer(out):
      provider = m.group("provider").lower()
      lat = float(m.group("lat"))
      lon = float(m.group("lon"))
      rest = m.group("rest")

      if abs(lat) < 0.001 and abs(lon) < 0.001:
        continue
      if abs(lat) > 90 or abs(lon) > 180:
        continue

      acc = find_num(rest, ["hAcc", "acc"], 25.0)
      alt = find_num(rest, ["alt"], 0.0)
      speed = find_num(rest, ["vel", "speed"], 0.0)
      bearing = find_num(rest, ["bear", "bearing"], 0.0)
      vacc = find_num(rest, ["vAcc"], max(acc * 1.5, 10.0))
      sacc = find_num(rest, ["sAcc"], 1.0)
      bacc = find_num(rest, ["bAcc"], 10.0)

      score = 2 if provider == "gps" else 1 if provider in ("net", "fused", "network") else 0

      item = {
        "score": score,
        "provider": provider,
        "lat": lat,
        "lon": lon,
        "acc": max(acc, 1.0),
        "alt": alt,
        "speed": max(speed, 0.0),
        "bearing": bearing % 360.0,
        "vacc": max(vacc, 1.0),
        "sacc": max(sacc, 0.1),
        "bacc": max(bacc, 1.0),
      }

      if best is None or item["score"] > best["score"]:
        best = item

  return best

def publish_fix(pm, fix):
  msg = messaging.new_message("gpsLocationExternal")
  g = msg.gpsLocationExternal

  g.flags = 1
  g.latitude = fix["lat"]
  g.longitude = fix["lon"]
  g.altitude = fix["alt"]
  g.speed = fix["speed"]
  g.bearingDeg = fix["bearing"]
  g.accuracy = fix["acc"]
  g.verticalAccuracy = fix["vacc"]
  g.speedAccuracy = fix["sacc"]
  g.bearingAccuracyDeg = fix["bacc"]
  g.timestamp = int(time.time() * 1000)

  br = math.radians(fix["bearing"])
  g.vNED = [
    fix["speed"] * math.cos(br),
    fix["speed"] * math.sin(br),
    0.0,
  ]

  pm.send("gpsLocationExternal", msg)

def main():
  pm = messaging.PubMaster(["gpsLocationExternal"])
  last_fix = None
  last_poll = 0
  last_print = 0

  while True:
    now = time.monotonic()

    if now - last_poll >= 1.0:
      last_poll = now
      fix = get_android_location()
      if fix is not None:
        last_fix = fix

    if last_fix is not None:
      publish_fix(pm, last_fix)

      if now - last_print >= 5.0:
        last_print = now
        print(
          "phonegpsd:",
          last_fix["provider"],
          last_fix["lat"],
          last_fix["lon"],
          "acc",
          last_fix["acc"],
          flush=True,
        )

    time.sleep(0.1)

if __name__ == "__main__":
  main()
