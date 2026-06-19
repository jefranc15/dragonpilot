#!/usr/bin/env python3
import time
from cereal import messaging

LAT = 14.9983074
LON = 120.783442

pm = messaging.PubMaster(["gpsLocationExternal"])

while True:
  msg = messaging.new_message("gpsLocationExternal")
  g = msg.gpsLocationExternal
  g.flags = 1
  g.latitude = LAT
  g.longitude = LON
  g.altitude = 30.0
  g.speed = 0.0
  g.bearingDeg = 0.0
  g.accuracy = 5.0
  g.verticalAccuracy = 10.0
  g.speedAccuracy = 1.0
  g.bearingAccuracyDeg = 10.0
  g.timestamp = int(time.time() * 1000)
  g.vNED = [0.0, 0.0, 0.0]
  pm.send("gpsLocationExternal", msg)
  time.sleep(0.1)
