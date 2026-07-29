#!/usr/bin/env python3
from __future__ import print_function

import os
import py_compile
import re
import shutil

PATH = "/data/openpilot/selfdrive/car/dnga/carcontroller.py"
BACKUP = PATH + ".before_steer_debug"
MARKER = "# DNGA_STEER_DEBUG_HOOK_BEGIN"


def indent(block, spaces):
  return "\n".join(spaces + line if line else line
                   for line in block.splitlines())


if not os.path.isfile(PATH):
  raise SystemExit("Not found: " + PATH)

with open(PATH, "r") as f:
  original = f.read()

if MARKER in original:
  print("Steering debug hook is already installed.")
  raise SystemExit(0)

if not os.path.exists(BACKUP):
  shutil.copy2(PATH, BACKUP)

text = original

# Add standard-library imports.
lines = text.splitlines(True)
insert_at = 0

while insert_at < len(lines):
  s = lines[insert_at].strip()

  if not s or s.startswith("#!"):
    insert_at += 1
    continue

  if s.startswith("from __future__ import"):
    insert_at += 1
    continue

  break

lines.insert(
  insert_at,
  "# DNGA_STEER_DEBUG_IMPORTS\n"
  "import json\n"
  "import socket\n"
)

text = "".join(lines)
lines = text.splitlines()

# Create the UDP socket after a steering-state initialization line.
init_index = None
init_spaces = None

patterns = (
  r"^\s*self\.last_steer\s*=",
  r"^\s*self\.apply_steer_last\s*=",
  r"^\s*self\.last_steer_cmd\s*=",
)

for i, line in enumerate(lines):
  if any(re.match(p, line) for p in patterns):
    init_index = i + 1
    init_spaces = re.match(r"^(\s*)", line).group(1)
    break

if init_index is None:
  raise SystemExit(
    "Could not find steering initialization. "
    "No changes were written."
  )

init_block = """# DNGA_STEER_DEBUG_INIT_BEGIN
try:
  self._dnga_debug_sock = socket.socket(
    socket.AF_INET, socket.SOCK_DGRAM)
  self._dnga_debug_sock.setblocking(False)
except Exception:
  self._dnga_debug_sock = None
# DNGA_STEER_DEBUG_INIT_END"""

lines[init_index:init_index] = indent(
  init_block, init_spaces).splitlines()

# Find the outbound steering CAN append.
send_index = None
send_spaces = None

for i, line in enumerate(lines):
  if "can_sends.append" not in line:
    continue

  nearby = " ".join(lines[i:min(i + 6, len(lines))]).lower()

  if (
    "steer" in nearby or
    "steering" in nearby or
    "2e4" in nearby
  ):
    send_index = i
    send_spaces = re.match(r"^(\s*)", line).group(1)
    break

if send_index is None:
  raise SystemExit(
    "Could not locate steering CAN send. "
    "No changes were written."
  )

hook = """# DNGA_STEER_DEBUG_HOOK_BEGIN
try:
  _d = locals()
  _cc = _d.get("CC", None)
  _cs = _d.get("CS", None)
  _out = getattr(_cs, "out", _cs)

  _act = _d.get("actuators", None)
  if _act is None and _cc is not None:
    _act = getattr(_cc, "actuators", None)

  _enabled = _d.get(
    "enabled",
    getattr(_cc, "enabled", False)
  )

  _frame = _d.get("frame", 0)

  _steer_max = _d.get(
    "steer_max",
    _d.get(
      "steer_max_interp",
      _d.get("dynamic_steer_max", 0.0)
    )
  )

  _requested = _d.get(
    "new_steer",
    _d.get(
      "requested_steer",
      _d.get("apply_torque_before_limits", None)
    )
  )

  if _requested is None:
    _requested = (
      float(getattr(_act, "steer", 0.0)) *
      float(_steer_max)
    )

  _applied = _d.get(
    "apply_steer",
    _d.get(
      "apply_torque",
      _d.get("steer", 0)
    )
  )

  _last = getattr(
    self,
    "last_steer",
    getattr(
      self,
      "apply_steer_last",
      getattr(self, "last_steer_cmd", 0)
    )
  )

  _steer_req = _d.get(
    "steer_req",
    _d.get("steer_request", _enabled)
  )

  _packet = {
    "frame": int(_frame),
    "enabled": bool(_enabled),
    "vEgo": float(getattr(_out, "vEgo", 0.0)),
    "actuatorSteer": float(
      getattr(_act, "steer", 0.0)
    ),
    "steerMax": float(_steer_max),
    "requestedSteer": int(round(float(_requested))),
    "lastSteer": int(round(float(_last))),
    "appliedSteer": int(round(float(_applied))),
    "driverTorque": float(
      getattr(_out, "steeringTorque", 0.0)
    ),
    "epsTorque": float(
      getattr(_out, "steeringTorqueEps", 0.0)
    ),
    "steeringPressed": bool(
      getattr(_out, "steeringPressed", False)
    ),
    "steerReq": bool(_steer_req),
    "rateLimited": bool(
      getattr(
        self,
        "steer_rate_limited",
        _requested != _applied
      )
    ),
  }

  if self._dnga_debug_sock is not None:
    self._dnga_debug_sock.sendto(
      json.dumps(
        _packet,
        separators=(",", ":")
      ).encode("utf-8"),
      ("127.0.0.1", 8061)
    )
except Exception:
  pass
# DNGA_STEER_DEBUG_HOOK_END"""

lines[send_index:send_index] = indent(
  hook, send_spaces).splitlines()

new_text = "\n".join(lines) + "\n"

with open(PATH, "w") as f:
  f.write(new_text)

try:
  py_compile.compile(PATH, doraise=True)
except Exception as e:
  shutil.copy2(BACKUP, PATH)
  raise SystemExit(
    "Compile failed. Original restored: %s" % e
  )

print("Installed steering debug hook successfully.")
print("Backup:", BACKUP)
print("CarController compile passed.")
