#!/usr/bin/env python3

from pathlib import Path
import py_compile
import re
import shutil
import sys

path = Path(
  "/data/openpilot/selfdrive/car/dnga/carcontroller.py"
)

backup = Path(
  "/data/openpilot/selfdrive/car/dnga/"
  "carcontroller.py.before_v25d"
)

marker = "# DNGA V2.5D EARLIER BRAKE ACTIVATION"

if not path.is_file():
  sys.exit("ERROR: carcontroller.py was not found.")

original = path.read_text()

if marker in original:
  print("v2.5d is already installed.")
  sys.exit(0)

changes = [
  (
    r"(?m)^(\s*)if brake_request > 0\.30:",
    r"\1if brake_request > 0.25:"
  ),
  (
    r"(?m)^(\s*)if not self\.brake_active "
    r"and self\.brake_req_counter >= 4:",
    r"\1if not self.brake_active "
    r"and self.brake_req_counter >= 3:"
  ),
]

updated = original

for pattern, replacement in changes:
  matches = re.findall(pattern, updated)

  if len(matches) != 1:
    sys.exit(
      "ERROR: Expected exactly one match for:\n"
      + pattern
      + "\nFound: "
      + str(len(matches))
      + "\nNo changes were written."
    )

  updated = re.sub(
    pattern,
    replacement,
    updated,
    count=1,
  )

comment_pattern = (
  r"(?m)^(\s*)# Engage after ~0\.20s at 20 Hz, "
  r"release after ~0\.10s\.$"
)

comment_matches = re.findall(
  comment_pattern,
  updated,
)

if len(comment_matches) == 1:
  updated = re.sub(
    comment_pattern,
    lambda match: (
      match.group(1)
      + marker
      + "\n"
      + match.group(1)
      + "# Engage after ~0.15s at 20 Hz; "
      + "release remains ~0.10s."
    ),
    updated,
    count=1,
  )
else:
  # Add the marker immediately before the activation condition.
  activation_line = (
    "if not self.brake_active "
    "and self.brake_req_counter >= 3:"
  )

  index = updated.find(activation_line)

  if index < 0:
    sys.exit(
      "ERROR: Could not insert the v2.5d marker. "
      "No changes were written."
    )

  line_start = updated.rfind("\n", 0, index) + 1
  indentation = updated[line_start:index]

  updated = (
    updated[:line_start]
    + indentation
    + marker
    + "\n"
    + updated[line_start:]
  )

# Confirm brake magnitudes still exist before writing.
for magnitude in ("1219", "1215", "1212"):
  if magnitude not in updated:
    sys.exit(
      "ERROR: Expected brake magnitude "
      + magnitude
      + " was not found. No changes were written."
    )

if not backup.exists():
  shutil.copy2(path, backup)
  print("Created backup:", backup)
else:
  print("Existing backup kept:", backup)

path.write_text(updated)

try:
  py_compile.compile(
    str(path),
    doraise=True,
  )
except Exception as error:
  shutil.copy2(backup, path)
  sys.exit(
    "ERROR: Compilation failed. "
    "The v2.5c backup was restored.\n"
    + str(error)
  )

print()
print("DNGA v2.5d installed successfully.")
print("Changed:")
print("  brake activation threshold: 0.30 -> 0.25")
print("  activation counter:         4 -> 3")
print()
print("Unchanged:")
print("  brake magnitudes:           1219 / 1215 / 1212")
print("  pump reaction:              -0.4")
print("  release threshold/counter")
print("  post-engagement brake block")
print("  new-lead cut-in protection")
print("  acceleration guard")
print()
print("carcontroller.py compilation passed.")
