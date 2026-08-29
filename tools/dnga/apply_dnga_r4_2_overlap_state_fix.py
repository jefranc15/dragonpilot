#!/usr/bin/env python3
from pathlib import Path
import py_compile

PATH = Path("selfdrive/car/dnga/carcontroller.py")
old_text = PATH.read_text()
new_text = old_text

replacements = [
(
'''    self.v33r4_decel_entry_frame = -1000000\n    self.v33r4_decel_entry_torque = 0\n    self.v33r4_decel_torque_cleared = False\n    self.v33r4_positive_overlap_counter = 0\n''',
'''    self.v33r4_decel_entry_frame = -1000000\n    self.v33r4_decel_entry_torque = 0\n    self.v33r4_decel_torque_cleared = False\n    # R4.2 keeps physical friction/positive-torque overlap timing independent\n    # from the decel-latch entry timestamp. Sharing these states caused the\n    # pre-friction planner interval to leak back into the overlap age.\n    self.v33r4_overlap_entry_frame = -1000000\n    self.v33r4_overlap_entry_torque = 0\n    self.v33r4_overlap_torque_cleared = False\n    self.v33r4_positive_overlap_counter = 0\n'''
),
(
'''    self.v33r4_brake_clear_counter = 0\n    self.v33r4_torque_ready_counter = 0\n    self.v33r4_positive_overlap_counter = 0\n    self.v25l_speed_offset = 0.0\n''',
'''    self.v33r4_brake_clear_counter = 0\n    self.v33r4_torque_ready_counter = 0\n    self.v33r4_overlap_entry_frame = -1000000\n    self.v33r4_overlap_entry_torque = 0\n    self.v33r4_overlap_torque_cleared = False\n    self.v33r4_positive_overlap_counter = 0\n    self.v25l_speed_offset = 0.0\n'''
),
(
'''      self.v33r4_decel_entry_frame = -1000000\n      self.v33r4_decel_entry_torque = 0\n      self.v33r4_decel_torque_cleared = False\n      self.v33r4_positive_overlap_counter = 0\n''',
'''      self.v33r4_decel_entry_frame = -1000000\n      self.v33r4_decel_entry_torque = 0\n      self.v33r4_decel_torque_cleared = False\n      self.v33r4_overlap_entry_frame = -1000000\n      self.v33r4_overlap_entry_torque = 0\n      self.v33r4_overlap_torque_cleared = False\n      self.v33r4_positive_overlap_counter = 0\n'''
),
(
'''      if not r4_negative_intent:\n        self.v33r4_decel_entry_frame = -1000000\n        self.v33r4_decel_entry_torque = 0\n        self.v33r4_decel_torque_cleared = False\n      elif r4_feedback_clean and r4_feedback["torque_actual"] <= 80:\n        # Once the powertrain has crossed through the positive-torque region,\n        # any later return to voted propulsion under friction is not an entry\n        # transient and should fault immediately.\n        self.v33r4_decel_torque_cleared = True\n\n      if (\n        r4_positive_under_friction and\n        self.v33r4_decel_entry_frame < 0\n      ):\n        self.v33r4_decel_entry_frame = frame\n        self.v33r4_decel_entry_torque = r4_feedback["torque_actual"]\n\n      r4_overlap_started = self.v33r4_decel_entry_frame >= 0\n      r4_overlap_age = (\n        frame - self.v33r4_decel_entry_frame\n        if r4_overlap_started else 0\n      )\n      r4_overlap_rising = (\n        r4_positive_under_friction and\n        r4_overlap_started and\n        r4_feedback["torque_actual"] >\n        max(80, self.v33r4_decel_entry_torque + V33R4_ENTRY_TORQUE_RISE_RAW)\n      )\n      r4_overlap_unsafe = (\n        r4_positive_under_friction and\n        (\n          not r4_negative_intent or\n          self.v33r4_decel_torque_cleared or\n          (\n            r4_overlap_started and\n            r4_overlap_age > V33R4_ENTRY_OVERLAP_FRAMES\n          ) or\n          r4_overlap_rising\n        )\n      )\n''',
'''      if not r4_negative_intent:\n        self.v33r4_overlap_entry_frame = -1000000\n        self.v33r4_overlap_entry_torque = 0\n        self.v33r4_overlap_torque_cleared = False\n      elif r4_feedback_clean and r4_feedback["torque_actual"] <= 80:\n        # Once the powertrain has crossed through the positive-torque region,\n        # any later return to voted propulsion under friction is not an entry\n        # transient and should fault immediately. Keep this independent from\n        # decel-latch clearing so a release-stage transition cannot reset it.\n        self.v33r4_overlap_torque_cleared = True\n\n      if (\n        r4_positive_under_friction and\n        self.v33r4_overlap_entry_frame < 0\n      ):\n        self.v33r4_overlap_entry_frame = frame\n        self.v33r4_overlap_entry_torque = r4_feedback["torque_actual"]\n\n      r4_overlap_started = self.v33r4_overlap_entry_frame >= 0\n      r4_overlap_age = (\n        frame - self.v33r4_overlap_entry_frame\n        if r4_overlap_started else 0\n      )\n      r4_overlap_rising = (\n        r4_positive_under_friction and\n        r4_overlap_started and\n        r4_feedback["torque_actual"] >\n        max(80, self.v33r4_overlap_entry_torque + V33R4_ENTRY_TORQUE_RISE_RAW)\n      )\n      r4_overlap_unsafe = (\n        r4_positive_under_friction and\n        (\n          not r4_negative_intent or\n          self.v33r4_overlap_torque_cleared or\n          (\n            r4_overlap_started and\n            r4_overlap_age > V33R4_ENTRY_OVERLAP_FRAMES\n          ) or\n          r4_overlap_rising\n        )\n      )\n'''
),
(
'''        self.v33r4_decel_entry_frame = -1000000\n        self.v33r4_decel_torque_cleared = False\n        self.v33r4_positive_overlap_counter = 0\n        self.v33r2_release_pump_until_frame = frame\n''',
'''        self.v33r4_decel_entry_frame = -1000000\n        self.v33r4_decel_torque_cleared = False\n        self.v33r4_overlap_entry_frame = -1000000\n        self.v33r4_overlap_entry_torque = 0\n        self.v33r4_overlap_torque_cleared = False\n        self.v33r4_positive_overlap_counter = 0\n        self.v33r2_release_pump_until_frame = frame\n'''
),
]

for i, (old, new) in enumerate(replacements, 1):
    count = new_text.count(old)
    if count != 1:
        raise SystemExit(f"replacement {i}: expected 1 match, found {count}")
    new_text = new_text.replace(old, new, 1)

PATH.write_text(new_text)
py_compile.compile(str(PATH), doraise=True)
assert 'self.v33r4_overlap_entry_frame = frame' in new_text
assert 'frame - self.v33r4_overlap_entry_frame' in new_text
assert 'self.v33r4_decel_entry_frame = frame' in new_text
print("R4.2 overlap state separated and py_compile passed")
