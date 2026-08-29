#!/usr/bin/env python3
from pathlib import Path
import py_compile

CC = Path("selfdrive/car/dnga/carcontroller.py")
CS = Path("selfdrive/car/dnga/carstate.py")

cc = CC.read_text()
cs = CS.read_text()

# Current nightly already contains the functional SET-under-gas, relaxed
# session-level rearm threshold, and physical-overlap timing work. V4.0 names
# that combined release and adds the final two pieces required by the road logs:
# preserve lateral on a longitudinal-only hybrid fault, and expose a physical
# SET/RES release edge so the fault can rearm without first killing engagement.

replacements = [
(
"""# V3.3R4 hybrid-feedback-supervisor build: offline/replay/bench validation.\n#\n""",
"""# V4.0 DNGA engagement and hybrid-feedback build.\n#\n# V4.0 combines the R4.1 stock-derived handoff with the 2026-08-29 road-log\n# fixes: SET/RES remains an enabled ACC session during driver gas override,\n# longitudinal fault rearm requires fresh/consistent brake-clear feedback but\n# leaves torque neutrality to the independent propulsion gate, the 0.55-second\n# positive-torque/friction allowance starts at actual physical overlap rather\n# than planner decel intent, and a longitudinal-only hybrid fault no longer\n# tears down otherwise healthy lateral steering.\n#\n"""
),
(
"""    self.v33r4_positive_overlap_counter = 0\n    self.v25l_speed_offset = 0.0\n    if hasattr(CS, \"is_cruise_latch\"):\n      CS.is_cruise_latch = False\n    CS.hybrid_feedback_fault = True\n""",
"""    self.v33r4_positive_overlap_counter = 0\n    self.v25l_speed_offset = 0.0\n    # V4.0: hybrid feedback supervises longitudinal actuation only. Keep the\n    # cruise latch and lateral session alive; longitudinal_session_allowed\n    # below still goes false while this fault is latched, so 0x271/0x273 fail\n    # non-propulsive without dropping a healthy 0x1D0 STEER_REQ.\n    CS.hybrid_feedback_fault = True\n"""
),
(
"""      # A fault is not cleared by timers or by the stale outer enabled flag.\n      # A new SET/RES engagement edge may clear it only while all observed\n      # feedback is fresh, mutually consistent, brake-clear, and non-positive.\n      if engagement_edge and self.v33r4_fault_latched:\n        if r4_rearm_ok:\n          self.v33r4_fault_latched = False\n          self.v33r4_fault_reason = \"\"\n        else:\n          self._v33r4_latch_fault(CS, \"feedback_not_safe_to_rearm\")\n""",
"""      # V4.0: because a longitudinal fault no longer destroys the cruise\n      # latch/lateral session, outer `enabled` may stay true. Accept either the\n      # normal outer engagement edge or a real physical SET/RES release edge\n      # from CarState as the explicit driver request to rearm longitudinal.\n      v40_rearm_edge = (\n        engagement_edge or\n        bool(getattr(CS, \"v40_acc_rearm_edge\", False))\n      )\n      if v40_rearm_edge and self.v33r4_fault_latched:\n        if r4_rearm_ok:\n          self.v33r4_fault_latched = False\n          self.v33r4_fault_reason = \"\"\n        else:\n          self._v33r4_latch_fault(CS, \"feedback_not_safe_to_rearm\")\n"""
),
]

for i, (old, new) in enumerate(replacements, 1):
    count = cc.count(old)
    if count != 1:
        raise SystemExit(f"carcontroller replacement {i}: expected 1 match, found {count}")
    cc = cc.replace(old, new, 1)

# Name the already-landed road-log fixes as V4.0 in source comments. These are
# comment-only substitutions; identifiers and behavior remain unchanged.
if "R4.2" not in cc:
    raise SystemExit("expected current R4.2 source comments before V4.0 rename")
cc = cc.replace("R4.2", "V4.0")

cs_replacements = [
(
"""    self.is_plus_btn_latch = False\n    self.is_minus_btn_latch = False\n    self.prev_distance_btn = False\n""",
"""    self.is_plus_btn_latch = False\n    self.is_minus_btn_latch = False\n    # V4.0 one-cycle SET/RES release edge consumed by CarController when a\n    # longitudinal-only hybrid feedback fault needs explicit driver rearm.\n    self.v40_acc_rearm_edge = False\n    self.prev_distance_btn = False\n"""
),
(
"""    minus_button = bool(cp.vl[\"PCM_BUTTONS\"][\"SET_MINUS\"])\n    plus_button = bool(cp.vl[\"PCM_BUTTONS\"][\"RES_PLUS\"])\n\n    if self.is_cruise_latch:\n""",
"""    minus_button = bool(cp.vl[\"PCM_BUTTONS\"][\"SET_MINUS\"])\n    plus_button = bool(cp.vl[\"PCM_BUTTONS\"][\"RES_PLUS\"])\n\n    # V4.0: previous latch values are updated later in this block, so this is\n    # true for exactly one CarState cycle on the physical SET or RES release.\n    self.v40_acc_rearm_edge = bool(\n      (self.is_plus_btn_latch and not plus_button) or\n      (self.is_minus_btn_latch and not minus_button)\n    )\n\n    if self.is_cruise_latch:\n"""
),
]

for i, (old, new) in enumerate(cs_replacements, 1):
    count = cs.count(old)
    if count != 1:
        raise SystemExit(f"carstate replacement {i}: expected 1 match, found {count}")
    cs = cs.replace(old, new, 1)

CC.write_text(cc)
CS.write_text(cs)
py_compile.compile(str(CC), doraise=True)
py_compile.compile(str(CS), doraise=True)
print("DNGA V4.0 patch applied; carcontroller.py and carstate.py py_compile PASS")
