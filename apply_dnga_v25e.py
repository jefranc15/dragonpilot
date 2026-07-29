#!/usr/bin/env python3

from pathlib import Path
import py_compile
import re
import shutil
import sys

PATH = Path(
  "/data/openpilot/selfdrive/car/dnga/carcontroller.py"
)

BACKUP = Path(
  "/data/openpilot/selfdrive/car/dnga/"
  "carcontroller.py.before_v25e"
)

MARKER = "# DNGA V2.5E OP-ONLY BRAKE REQUEST"

if not PATH.is_file():
  sys.exit("ERROR: carcontroller.py was not found.")

original = PATH.read_text()

if MARKER in original:
  print("DNGA v2.5e is already installed.")
  sys.exit(0)

# Require the expected v2.5d baseline.
required = [
  "if brake_request > 0.25:",
  "self.brake_req_counter >= 3",
  "brake_mag = 1219",
  "brake_mag = 1215",
  "brake_mag = 1212",
  "brake_request = max(apply_brake, overspeed_brake)",
  "accel_guard_ok = CS.out.aEgo < 0.05",
]

missing = [
  item for item in required
  if item not in original
]

if missing:
  print("ERROR: Current controller is not the expected v2.5d baseline.")
  print("Missing:")
  for item in missing:
    print("  " + item)
  print("No changes were written.")
  sys.exit(1)

updated = original

# Remove the complete independent overspeed-brake section.
overspeed_pattern = re.compile(
  r"(?ms)"
  r"^(?P<indent>[ \t]*)"
  r"# apply_brake = OP requested decel/brake\.\n"
  r"(?P=indent)"
  r"# overspeed_brake = extra gentle brake "
  r"when car is above set speed\.\n"
  r".*?"
  r"^(?P=indent)"
  r"brake_request = max\(apply_brake, overspeed_brake\)"
  r"[^\n]*$"
)

overspeed_matches = list(
  overspeed_pattern.finditer(updated)
)

if len(overspeed_matches) != 1:
  sys.exit(
    "ERROR: Expected exactly one overspeed-brake block; "
    "found %d. No changes were written."
    % len(overspeed_matches)
  )

def replace_overspeed(match):
  indent = match.group("indent")
  return (
    indent + MARKER + "\n"
    + indent
    + "# Brake request comes only from negative "
      "openpilot acceleration.\n"
    + indent
    + "# No independent set-speed/overspeed brake helper.\n"
    + indent
    + "brake_request = apply_brake"
  )

updated = overspeed_pattern.sub(
  replace_overspeed,
  updated,
  count=1,
)

# Remove the measured-acceleration guard and its stale comments.
guard_block_pattern = re.compile(
  r"(?m)"
  r"^[ \t]*# Keep the anti-jerk aEgo guard[^\n]*\n"
  r"^[ \t]*# OP/overspeed[^\n]*\n"
  r"^[ \t]*accel_guard_ok = "
  r"CS\.out\.aEgo < 0\.05[^\n]*\n"
)

guard_matches = list(
  guard_block_pattern.finditer(updated)
)

if len(guard_matches) != 1:
  sys.exit(
    "ERROR: Expected exactly one aEgo guard block; "
    "found %d. No changes were written."
    % len(guard_matches)
  )

updated = guard_block_pattern.sub(
  "",
  updated,
  count=1,
)

# Replace the measured-response guard with a command-domain interlock.
interlock_pattern = re.compile(
  r"(?m)"
  r"^(?P<indent>[ \t]*)"
  r"accel_guard_ok and[^\n]*$"
)

interlock_matches = list(
  interlock_pattern.finditer(updated)
)

if len(interlock_matches) != 1:
  sys.exit(
    "ERROR: Expected exactly one accel_guard_ok condition; "
    "found %d. No changes were written."
    % len(interlock_matches)
  )

updated = interlock_pattern.sub(
  lambda match: (
    match.group("indent")
    + "apply_accel < 0.0 and"
    + "  # V2.5e: never brake during positive OP accel"
  ),
  updated,
  count=1,
)

# Safety verification before writing.
checks = {
  "v2.5e marker": MARKER in updated,
  "OP-only brake request":
    "brake_request = apply_brake" in updated,
  "command interlock":
    "apply_accel < 0.0 and" in updated,
  "overspeed helper removed":
    "overspeed_brake" not in updated,
  "aEgo guard removed":
    "accel_guard_ok" not in updated,
  "threshold remains 0.25":
    "if brake_request > 0.25:" in updated,
  "counter remains 3":
    "self.brake_req_counter >= 3" in updated,
  "magnitude 1219 unchanged":
    "brake_mag = 1219" in updated,
  "magnitude 1215 unchanged":
    "brake_mag = 1215" in updated,
  "magnitude 1212 unchanged":
    "brake_mag = 1212" in updated,
}

failed = [
  name for name, passed in checks.items()
  if not passed
]

if failed:
  print("ERROR: Verification failed:")
  for name in failed:
    print("  " + name)
  print("No changes were written.")
  sys.exit(1)

if not BACKUP.exists():
  shutil.copy2(PATH, BACKUP)
  print("Created backup:", BACKUP)
else:
  print("Existing backup retained:", BACKUP)

PATH.write_text(updated)

try:
  py_compile.compile(
    str(PATH),
    doraise=True,
  )
except Exception as error:
  shutil.copy2(BACKUP, PATH)
  sys.exit(
    "ERROR: Compilation failed. "
    "The previous controller was restored.\n%s"
    % error
  )

print()
print("DNGA v2.5e installed successfully.")
print()
print("Changed:")
print("  Removed independent overspeed brake")
print("  Removed measured aEgo brake guard")
print("  Added apply_accel < 0 command interlock")
print()
print("Unchanged:")
print("  activation threshold: 0.25")
print("  activation counter:   3")
print("  brake magnitudes:      1219 / 1215 / 1212")
print("  post-engagement block")
print("  new-lead cut-in protection")
print("  release hysteresis")
print("  pump reaction")
print()
print("carcontroller.py compile passed.")
