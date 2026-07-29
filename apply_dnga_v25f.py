#!/usr/bin/env python3

from pathlib import Path
import py_compile
import shutil
import sys

PATH = Path(
  "/data/openpilot/selfdrive/car/dnga/carcontroller.py"
)

BACKUP = Path(
  "/data/openpilot/selfdrive/car/dnga/"
  "carcontroller.py.before_v25f"
)

MARKER = "# DNGA V2.5F CONTINUOUS DESIRED SPEED"

if not PATH.is_file():
  sys.exit("ERROR: carcontroller.py was not found.")

original = PATH.read_text()

if MARKER in original:
  print("DNGA v2.5f is already installed.")
  sys.exit(0)

# Confirm that the current controller is the expected v2.5e baseline.
required = [
  "# DNGA V2.5E OP-ONLY BRAKE REQUEST",
  "brake_request = apply_brake",
  "apply_accel < 0.0 and",
  "if brake_request > 0.25:",
  "self.brake_req_counter >= 3",
  "brake_mag = 1219",
  "brake_mag = 1215",
  "brake_mag = 1212",
]

missing = [
  item for item in required
  if item not in original
]

if missing:
  print("ERROR: The file is not the expected v2.5e baseline.")
  print("Missing:")
  for item in missing:
    print("  " + item)
  print("No changes were written.")
  sys.exit(1)

if "overspeed_brake" in original:
  sys.exit(
    "ERROR: overspeed_brake still exists. "
    "No changes were written."
  )

if "accel_guard_ok" in original:
  sys.exit(
    "ERROR: accel_guard_ok still exists. "
    "No changes were written."
  )

lines = original.splitlines()

start_index = None
end_index = None

for index, line in enumerate(lines):
  if "boost = interp(CS.out.vEgo" in line:
    if start_index is not None:
      sys.exit(
        "ERROR: More than one old desired-speed block found."
      )
    start_index = index

if start_index is None:
  sys.exit(
    "ERROR: Could not find the old boost/des_speed block. "
    "No changes were written."
  )

for index in range(start_index, len(lines)):
  if (
    "des_speed = max(0.0, base_speed + clip(" in
    lines[index]
  ):
    end_index = index
    break

if end_index is None:
  sys.exit(
    "ERROR: Could not find the end of the old "
    "desired-speed block. No changes were written."
  )

old_block = "\n".join(
  lines[start_index:end_index + 1]
)

expected_old_parts = [
  "if enabled and actuators.accel > 0.05:",
  "des_speed = max(CS.out.vEgo, CS.cruise_speed)",
  "base_speed = getattr(actuators, 'speed', CS.out.vEgo)",
]

missing_old_parts = [
  item for item in expected_old_parts
  if item not in old_block
]

if missing_old_parts:
  print(
    "ERROR: The desired-speed block does not match "
    "the expected version."
  )
  print("Missing:")
  for item in missing_old_parts:
    print("  " + item)
  print("No changes were written.")
  sys.exit(1)

indent = lines[start_index][
  :len(lines[start_index]) -
   len(lines[start_index].lstrip())
]

new_block = [
  indent + MARKER,
  indent + (
    "# Avoid 0/full-cruise ACC_CMD jumps when "
    "acceleration crosses 0.05."
  ),
  indent + (
    "# Continuous formula based on Bukapilot staging."
  ),
  indent + (
    "t_lookup = 0.35 + 0.07 * CS.out.vEgo"
  ),
  indent + (
    "des_speed = max("
  ),
  indent + (
    "  0.0,"
  ),
  indent + (
    "  CS.out.vEgo + apply_accel * t_lookup,"
  ),
  indent + (
    ")"
  ),
]

updated_lines = (
  lines[:start_index] +
  new_block +
  lines[end_index + 1:]
)

updated = "\n".join(updated_lines) + "\n"

# Verify that the old discontinuous logic is gone.
for forbidden in [
  "base_speed = getattr(actuators, 'speed'",
  "des_speed = max(CS.out.vEgo, CS.cruise_speed)",
  "if enabled and actuators.accel > 0.05:",
]:
  if forbidden in updated:
    sys.exit(
      "ERROR: Old desired-speed logic remains: "
      + forbidden
    )

# Verify that only the desired-speed behavior was changed.
checks = {
  "v2.5f marker":
    MARKER in updated,

  "continuous time lookup":
    "t_lookup = 0.35 + 0.07 * CS.out.vEgo"
    in updated,

  "continuous speed command":
    "CS.out.vEgo + apply_accel * t_lookup"
    in updated,

  "OP-only brake request unchanged":
    "brake_request = apply_brake" in updated,

  "negative-command interlock unchanged":
    "apply_accel < 0.0 and" in updated,

  "threshold unchanged":
    "if brake_request > 0.25:" in updated,

  "counter unchanged":
    "self.brake_req_counter >= 3" in updated,

  "1219 unchanged":
    "brake_mag = 1219" in updated,

  "1215 unchanged":
    "brake_mag = 1215" in updated,

  "1212 unchanged":
    "brake_mag = 1212" in updated,
}

failed = [
  name for name, passed in checks.items()
  if not passed
]

if failed:
  print("ERROR: Verification failed:")
  for item in failed:
    print("  " + item)
  print("No changes were written.")
  sys.exit(1)

# Save the exact current v2.5e file for rollback.
shutil.copy2(PATH, BACKUP)
print("Created backup:", BACKUP)

PATH.write_text(updated)

try:
  py_compile.compile(
    str(PATH),
    doraise=True,
  )
except Exception as error:
  PATH.write_text(original)
  sys.exit(
    "ERROR: Compilation failed. "
    "The original v2.5e file was restored.\n"
    + str(error)
  )

print()
print("DNGA v2.5f installed successfully.")
print()
print("Changed:")
print("  Removed 0/full-cruise desired-speed switching")
print("  Removed dependency on actuators.speed")
print("  Added continuous desired-speed calculation")
print()
print("Unchanged:")
print("  v2.5e OP-only brake request")
print("  activation threshold: 0.25")
print("  activation counter:   3")
print("  brake magnitudes:      1219 / 1215 / 1212")
print("  pump reaction:         -0.4")
print("  post-engagement block")
print("  cut-in protection")
print("  release hysteresis")
print()
print("carcontroller.py compilation passed.")
