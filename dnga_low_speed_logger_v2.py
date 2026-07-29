#!/usr/bin/env python3
from __future__ import print_function

import csv
import gzip
import json
import os
import socket
import time
from datetime import datetime

import cereal.messaging as messaging
from common.realtime import Ratekeeper


RATE = 50
PORT = 8061

SERVICES = [
  "carState",
  "carControl",
  "controlsState",
  "liveParameters",
]

CAN_IDS = [
  0x271,
  0x273,
  0x274,
  0x2E4,
  0x191,
  0x412,
  0x2E6,
  0x520,
]

BUSES = [0, 1, 2, 128]


def get(obj, path, default=None):
  try:
    value = obj

    for name in path.split("."):
      value = getattr(value, name)

    return value
  except Exception:
    return default


def num(value, default=float("nan")):
  try:
    return float(value)
  except Exception:
    return default


def flag(value, default=False):
  try:
    return bool(value)
  except Exception:
    return default


def hexdata(data):
  try:
    return bytes(data).hex().upper()
  except Exception:
    try:
      return "".join("%02X" % ord(x) for x in data)
    except Exception:
      return ""


def open_sock(name):
  try:
    return messaging.sub_sock(name, conflate=False)
  except TypeError:
    return messaging.sub_sock(name)


def drain_can(sock, field, cache):
  events = 0
  frames_seen = 0

  while True:
    try:
      msg = messaging.recv_one_or_none(sock)
    except Exception:
      msg = None

    if msg is None:
      break

    events += 1

    try:
      frames = getattr(msg, field)
    except Exception:
      continue

    now = time.monotonic()

    for frame in frames:
      try:
        address = int(frame.address)
        bus = int(frame.src)
      except Exception:
        continue

      if address not in CAN_IDS:
        continue

      cache[(address, bus)] = (
        hexdata(frame.dat),
        now,
      )

      frames_seen += 1

  return events, frames_seen


def drain_debug(sock, latest):
  count = 0

  while True:
    try:
      payload, _ = sock.recvfrom(8192)
    except Exception:
      break

    try:
      if not isinstance(payload, str):
        payload = payload.decode("utf-8", "replace")

      data = json.loads(payload)

      if isinstance(data, dict):
        latest.clear()
        latest.update(data)
        latest["_time"] = time.monotonic()
        count += 1
    except Exception:
      pass

  return count


def lateral_values(controls):
  result = {
    "latType": "",
    "latActive": False,
    "latOutput": float("nan"),
    "latSaturated": False,
    "desiredAngleDeg": float("nan"),
    "controllerAngleDeg": float("nan"),
    "angleErrorDeg": float("nan"),
    "latP": float("nan"),
    "latI": float("nan"),
    "latF": float("nan"),
  }

  state_union = get(
    controls,
    "lateralControlState",
    None
  )

  if state_union is None:
    return result

  try:
    state_name = state_union.which()
    state = getattr(state_union, state_name)
  except Exception:
    return result

  result["latType"] = state_name
  result["latActive"] = flag(
    get(state, "active", False)
  )
  result["latOutput"] = num(
    get(state, "output", float("nan"))
  )
  result["latSaturated"] = flag(
    get(state, "saturated", False)
  )
  result["desiredAngleDeg"] = num(
    get(
      state,
      "steeringAngleDesiredDeg",
      float("nan")
    )
  )
  result["controllerAngleDeg"] = num(
    get(
      state,
      "steeringAngleDeg",
      float("nan")
    )
  )
  result["angleErrorDeg"] = num(
    get(state, "angleError", float("nan"))
  )
  result["latP"] = num(
    get(state, "p", float("nan"))
  )
  result["latI"] = num(
    get(state, "i", float("nan"))
  )
  result["latF"] = num(
    get(state, "f", float("nan"))
  )

  return result


def can_values(prefix, cache):
  now = time.monotonic()
  values = {}

  for address in CAN_IDS:
    for bus in BUSES:
      name = "%s_%X_bus%d" % (
        prefix,
        address,
        bus,
      )

      item = cache.get((address, bus))

      if item is None:
        values[name] = ""
        values[name + "_ageMs"] = ""
      else:
        values[name] = item[0]
        values[name + "_ageMs"] = (
          now - item[1]
        ) * 1000.0

  return values


def fields():
  names = [
    "t",
    "wallTime",
    "rxEvents",
    "rxFrames",
    "txEvents",
    "txFrames",
    "debugPackets",

    "vEgo",
    "vEgoKph",
    "aEgo",
    "gasPressed",
    "brakePressed",
    "standstill",

    "cruiseEnabled",
    "cruiseAvailable",
    "cruiseStandstill",
    "cruiseSpeedKph",

    "steeringAngleDeg",
    "steeringRateDeg",
    "steeringTorque",
    "steeringTorqueEps",
    "steeringPressed",
    "leftBlinker",
    "rightBlinker",

    "actSteer",
    "actAccel",
    "actGas",
    "actBrake",
    "actCurvature",

    "controlsEnabled",
    "controlsActive",
    "controlsATarget",

    "latType",
    "latActive",
    "latOutput",
    "latSaturated",
    "desiredAngleDeg",
    "controllerAngleDeg",
    "angleErrorDeg",
    "latP",
    "latI",
    "latF",

    "liveValid",
    "steerRatioLive",
    "stiffnessFactor",
    "angleOffsetDeg",

    "debugAlive",
    "debugAgeMs",
    "ccFrame",
    "ccEnabled",
    "ccVEgoKph",
    "ccActuatorSteer",
    "ccSteerMax",
    "ccRequestedSteer",
    "ccLastSteer",
    "ccAppliedSteer",
    "ccAppliedPercent",
    "ccDriverTorque",
    "ccEpsTorque",
    "ccSteeringPressed",
    "ccSteerReq",
    "ccRateLimited",
  ]

  for prefix in ("rx", "tx"):
    for address in CAN_IDS:
      for bus in BUSES:
        name = "%s_%X_bus%d" % (
          prefix,
          address,
          bus,
        )

        names.append(name)
        names.append(name + "_ageMs")

  return names


def main():
  os.chdir("/data/openpilot")

  filename = (
    "dnga_low_speed_steer_v2_" +
    datetime.now().strftime("%m%d_%H%M%S") +
    ".csv.gz"
  )

  path = os.path.join(
    "/data/openpilot",
    filename,
  )

  sm = messaging.SubMaster(SERVICES)

  can_sock = open_sock("can")
  sendcan_sock = open_sock("sendcan")

  debug_sock = socket.socket(
    socket.AF_INET,
    socket.SOCK_DGRAM,
  )

  debug_sock.setsockopt(
    socket.SOL_SOCKET,
    socket.SO_REUSEADDR,
    1,
  )

  debug_sock.bind(("127.0.0.1", PORT))
  debug_sock.setblocking(False)

  rx_cache = {}
  tx_cache = {}
  debug = {}

  start = time.monotonic()
  last_print = start
  rows = 0

  print("DNGA steering logger v2")
  print("Output:", path)
  print("Press Ctrl+C after the drive.")

  rk = Ratekeeper(RATE)

  try:
    with gzip.open(
      path,
      "wt",
      newline=""
    ) as output:

      writer = csv.DictWriter(
        output,
        fieldnames=fields(),
        extrasaction="ignore",
      )

      writer.writeheader()

      while True:
        now = time.monotonic()

        sm.update(0)

        rx_events, rx_frames = drain_can(
          can_sock,
          "can",
          rx_cache,
        )

        tx_events, tx_frames = drain_can(
          sendcan_sock,
          "sendcan",
          tx_cache,
        )

        debug_packets = drain_debug(
          debug_sock,
          debug,
        )

        cs = sm["carState"]
        cc = sm["carControl"]
        controls = sm["controlsState"]
        live = sm["liveParameters"]

        cruise = get(cs, "cruiseState", None)
        act = get(cc, "actuators", None)

        debug_time = debug.get("_time", 0.0)

        if debug_time:
          debug_age = (
            time.monotonic() - debug_time
          ) * 1000.0
        else:
          debug_age = float("nan")

        debug_alive = (
          debug_time > 0 and
          debug_age < 250.0
        )

        steer_max = num(
          debug.get("steerMax", float("nan"))
        )

        applied = num(
          debug.get(
            "appliedSteer",
            float("nan")
          )
        )

        applied_percent = float("nan")

        if (
          steer_max == steer_max and
          abs(steer_max) > 0.001
        ):
          applied_percent = (
            100.0 * applied / steer_max
          )

        row = {
          "t": now - start,
          "wallTime": datetime.now().isoformat(),
          "rxEvents": rx_events,
          "rxFrames": rx_frames,
          "txEvents": tx_events,
          "txFrames": tx_frames,
          "debugPackets": debug_packets,

          "vEgo": num(get(cs, "vEgo", 0.0), 0.0),
          "vEgoKph": num(
            get(cs, "vEgo", 0.0),
            0.0
          ) * 3.6,
          "aEgo": num(get(cs, "aEgo", 0.0), 0.0),
          "gasPressed": flag(
            get(cs, "gasPressed", False)
          ),
          "brakePressed": flag(
            get(cs, "brakePressed", False)
          ),
          "standstill": flag(
            get(cs, "standstill", False)
          ),

          "cruiseEnabled": flag(
            get(cruise, "enabled", False)
          ),
          "cruiseAvailable": flag(
            get(cruise, "available", False)
          ),
          "cruiseStandstill": flag(
            get(cruise, "standstill", False)
          ),
          "cruiseSpeedKph": num(
            get(cruise, "speed", 0.0),
            0.0
          ) * 3.6,

          "steeringAngleDeg": num(
            get(cs, "steeringAngleDeg", 0.0),
            0.0
          ),
          "steeringRateDeg": num(
            get(cs, "steeringRateDeg", 0.0),
            0.0
          ),
          "steeringTorque": num(
            get(
              cs,
              "steeringTorque",
              float("nan")
            )
          ),
          "steeringTorqueEps": num(
            get(
              cs,
              "steeringTorqueEps",
              float("nan")
            )
          ),
          "steeringPressed": flag(
            get(cs, "steeringPressed", False)
          ),
          "leftBlinker": flag(
            get(cs, "leftBlinker", False)
          ),
          "rightBlinker": flag(
            get(cs, "rightBlinker", False)
          ),

          "actSteer": num(
            get(act, "steer", 0.0),
            0.0
          ),
          "actAccel": num(
            get(act, "accel", 0.0),
            0.0
          ),
          "actGas": num(
            get(act, "gas", 0.0),
            0.0
          ),
          "actBrake": num(
            get(act, "brake", 0.0),
            0.0
          ),
          "actCurvature": num(
            get(
              act,
              "curvature",
              float("nan")
            )
          ),

          "controlsEnabled": flag(
            get(controls, "enabled", False)
          ),
          "controlsActive": flag(
            get(controls, "active", False)
          ),
          "controlsATarget": num(
            get(
              controls,
              "aTarget",
              float("nan")
            )
          ),

          "liveValid": flag(
            get(live, "valid", False)
          ),
          "steerRatioLive": num(
            get(
              live,
              "steerRatio",
              float("nan")
            )
          ),
          "stiffnessFactor": num(
            get(
              live,
              "stiffnessFactor",
              float("nan")
            )
          ),
          "angleOffsetDeg": num(
            get(
              live,
              "angleOffsetDeg",
              float("nan")
            )
          ),

          "debugAlive": debug_alive,
          "debugAgeMs": debug_age,
          "ccFrame": debug.get("frame", ""),
          "ccEnabled": debug.get("enabled", ""),
          "ccVEgoKph": num(
            debug.get("vEgo", float("nan"))
          ) * 3.6,
          "ccActuatorSteer": debug.get(
            "actuatorSteer",
            ""
          ),
          "ccSteerMax": debug.get(
            "steerMax",
            ""
          ),
          "ccRequestedSteer": debug.get(
            "requestedSteer",
            ""
          ),
          "ccLastSteer": debug.get(
            "lastSteer",
            ""
          ),
          "ccAppliedSteer": debug.get(
            "appliedSteer",
            ""
          ),
          "ccAppliedPercent": applied_percent,
          "ccDriverTorque": debug.get(
            "driverTorque",
            ""
          ),
          "ccEpsTorque": debug.get(
            "epsTorque",
            ""
          ),
          "ccSteeringPressed": debug.get(
            "steeringPressed",
            ""
          ),
          "ccSteerReq": debug.get(
            "steerReq",
            ""
          ),
          "ccRateLimited": debug.get(
            "rateLimited",
            ""
          ),
        }

        row.update(lateral_values(controls))
        row.update(can_values("rx", rx_cache))
        row.update(can_values("tx", tx_cache))

        writer.writerow(row)
        rows += 1

        if rows % 250 == 0:
          output.flush()

        if now - last_print >= 5.0:
          fresh_buses = []

          for bus in BUSES:
            item = tx_cache.get((0x2E4, bus))

            if (
              item is not None and
              now - item[1] < 0.5
            ):
              fresh_buses.append(str(bus))

          print(
            "%.1fs rows=%d speed=%.1f "
            "debug=%s tx2E4=%s" % (
              row["t"],
              rows,
              row["vEgoKph"],
              "alive" if debug_alive else "MISSING",
              ",".join(fresh_buses)
              if fresh_buses else "none",
            )
          )

          last_print = now

        rk.keep_time()

  except KeyboardInterrupt:
    print("\nStopping logger...")

  finally:
    try:
      debug_sock.close()
    except Exception:
      pass

  print("Saved %d rows to:" % rows)
  print(path)


if __name__ == "__main__":
  main()
