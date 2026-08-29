#!/usr/bin/env python3
from pathlib import Path
import py_compile

CC = Path("selfdrive/car/dnga/carcontroller.py")
CS = Path("selfdrive/car/dnga/carstate.py")

cc = CC.read_text()
cs = CS.read_text()

cc_replacements = [
(
"""# V3.3R4 hybrid-feedback-supervisor build: offline/replay/bench validation.\n#\n""",
"""# V4.0 DNGA engagement/fault-separation build.\n#\n# V4.0 keeps the R4.1 stock-derived brake-to-propulsion handoff, then fixes\n# four issues proven by the 2026-08-29 road logs: SET while overriding with\n# gas now establishes the ACC session without allowing OP actuation; a latched\n# longitudinal hybrid fault can re-arm on a fresh SET/RES release once brake\n# feedback is clean; the positive-torque/friction overlap timer begins at the\n# first real overlap instead of planner decel intent; and a longitudinal-only\n# hybrid fault no longer clears the cruise latch and therefore no longer drops\n# otherwise healthy lateral steering.\n#\n"""
),
(
"""    self.v33r4_decel_entry_torque = 0\n    self.v33r4_decel_torque_cleared = False\n    self.v33r4_positive_overlap_counter = 0\n\n    self.v25r_plan_source = \"\"\n""",
"""    self.v33r4_decel_entry_torque = 0\n    self.v33r4_decel_torque_cleared = False\n    self.v33r4_positive_overlap_counter = 0\n\n    # V4.0 actual positive-torque + friction overlap clock. This is separate\n    # from the deceleration-intent clock because the 2026-08-29 logs showed\n    # 0.5-0.7 s between planner decel intent and physical friction onset.\n    self.v40_overlap_start_frame = -1000000\n    self.v40_overlap_start_torque = 0\n\n    self.v25r_plan_source = \"\"\n"""
),
(
"""    self.v33r4_positive_overlap_counter = 0\n    self.v25l_speed_offset = 0.0\n    if hasattr(CS, \"is_cruise_latch\"):\n      CS.is_cruise_latch = False\n    CS.hybrid_feedback_fault = True\n""",
"""    self.v33r4_positive_overlap_counter = 0\n    self.v40_overlap_start_frame = -1000000\n    self.v40_overlap_start_torque = 0\n    self.v25l_speed_offset = 0.0\n    # V4.0: this is a longitudinal actuator fault, not a steering fault. Fail\n    # longitudinal CAN non-propulsive below, but preserve the cruise/lateral\n    # session so a healthy STEER_REQ is not dropped by hybrid feedback logic.\n    CS.hybrid_feedback_fault = True\n"""
),
(
"""      self.v33r4_decel_entry_torque = 0\n      self.v33r4_decel_torque_cleared = False\n      self.v33r4_positive_overlap_counter = 0\n\n    self.prev_enabled = enabled  # Save enabled state for the next control cycle\n""",
"""      self.v33r4_decel_entry_torque = 0\n      self.v33r4_decel_torque_cleared = False\n      self.v33r4_positive_overlap_counter = 0\n      self.v40_overlap_start_frame = -1000000\n      self.v40_overlap_start_torque = 0\n\n    self.prev_enabled = enabled  # Save enabled state for the next control cycle\n"""
),
(
"""      base_control_allowed = (\n        enabled and\n        CS.out.cruiseState.enabled and\n        not pcm_cancel_cmd and\n        not CS.out.gasPressed and\n        not CS.out.brakePressed\n      )\n      r4_feedback = hybrid_feedback_snapshot(CS, frame)\n      r4_feedback_clean = (\n        r4_feedback[\"fresh\"] and r4_feedback[\"consistent\"]\n      )\n      r4_rearm_ok = (\n        r4_feedback_clean and\n        r4_feedback[\"brakes_clear\"] and\n        r4_feedback[\"torque_ramp_ready\"] and\n        not r4_feedback[\"positive_vote\"]\n      )\n\n      # A fault is not cleared by timers or by the stale outer enabled flag.\n      # A new SET/RES engagement edge may clear it only while all observed\n      # feedback is fresh, mutually consistent, brake-clear, and non-positive.\n      if engagement_edge and self.v33r4_fault_latched:\n        if r4_rearm_ok:\n          self.v33r4_fault_latched = False\n          self.v33r4_fault_reason = \"\"\n        else:\n          self._v33r4_latch_fault(CS, \"feedback_not_safe_to_rearm\")\n""",
"""      # V4.0 separates ACC-session state from actuator permission. Stock ACC\n      # accepts SET while the driver is overriding with the accelerator; OP\n      # should likewise publish the set speed/ready session immediately, while\n      # gas continues to prohibit autonomous braking and propulsion.\n      v40_session_allowed = (\n        enabled and\n        CS.out.cruiseState.enabled and\n        not pcm_cancel_cmd and\n        not CS.out.brakePressed\n      )\n      base_control_allowed = (\n        v40_session_allowed and\n        not CS.out.gasPressed\n      )\n      r4_feedback = hybrid_feedback_snapshot(CS, frame)\n      r4_feedback_clean = (\n        r4_feedback[\"fresh\"] and r4_feedback[\"consistent\"]\n      )\n\n      # V4.0 re-arm requires trustworthy brake-clear feedback, but does not\n      # require the powertrain torque to be neutral already. Torque neutrality\n      # remains mandatory later in r4_propulsion_ramp_ready before positive OP\n      # target buildup. This allows SET/RES while coasting or overriding gas.\n      r4_rearm_ok = (\n        r4_feedback_clean and\n        r4_feedback[\"brakes_clear\"]\n      )\n      v40_rearm_edge = (\n        engagement_edge or\n        bool(getattr(CS, \"v40_acc_rearm_edge\", False))\n      )\n\n      # A latched longitudinal fault clears only on a fresh SET/RES release (or\n      # the normal outer engagement edge) while the physical brake channels are\n      # fresh, mutually consistent, and clear.\n      if v40_rearm_edge and self.v33r4_fault_latched:\n        if r4_rearm_ok:\n          self.v33r4_fault_latched = False\n          self.v33r4_fault_reason = \"\"\n        else:\n          self._v33r4_latch_fault(CS, \"feedback_not_safe_to_rearm\")\n"""
),
(
"""      control_allowed = (\n        base_control_allowed and not self.v33r4_fault_latched\n      )\n      CS.hybrid_feedback_fault = self.v33r4_fault_latched\n""",
"""      control_allowed = (\n        base_control_allowed and not self.v33r4_fault_latched\n      )\n      v40_gas_override = (\n        v40_session_allowed and\n        CS.out.gasPressed and\n        not self.v33r4_fault_latched\n      )\n      CS.hybrid_feedback_fault = self.v33r4_fault_latched\n"""
),
(
"""        self.v33r4_decel_entry_frame = -1000000\n        self.v33r4_decel_torque_cleared = False\n        self.v33r4_positive_overlap_counter = 0\n        self.v33r2_release_pump_until_frame = frame\n""",
"""        self.v33r4_decel_entry_frame = -1000000\n        self.v33r4_decel_torque_cleared = False\n        self.v33r4_positive_overlap_counter = 0\n        self.v40_overlap_start_frame = -1000000\n        self.v40_overlap_start_torque = 0\n        self.v33r2_release_pump_until_frame = frame\n"""
),
(
"""      # Toyota briefly overlaps positive hybrid torque with brake entry. In\n      # the passive stock capture the longest voted positive-torque + friction\n      # overlap was 0.20 s. Permit a wider 0.55 s entry envelope only while\n      # torque does not rise materially; once torque has cleared, any return of\n      # voted propulsion under friction is a fault.\n      r4_negative_intent = (\n        control_allowed and\n        (\n          hydraulic_req or\n          release_pump_active or\n          self.v33r2_decel_latched or\n          (plan_fresh and planner_brake_request >= V33R2_DECEL_LATCH_BRAKE)\n        )\n      )\n      if r4_negative_intent and self.v33r4_decel_entry_frame < 0:\n        self.v33r4_decel_entry_frame = frame\n        self.v33r4_decel_entry_torque = r4_feedback[\"torque_actual\"]\n        self.v33r4_decel_torque_cleared = (\n          r4_feedback[\"torque_actual\"] <= 80\n        )\n      if r4_negative_intent and r4_feedback[\"torque_actual\"] <= 80:\n        self.v33r4_decel_torque_cleared = True\n\n      r4_positive_under_friction = (\n        control_allowed and\n        r4_feedback_clean and\n        r4_feedback[\"friction\"] > 0 and\n        r4_feedback[\"positive_vote\"]\n      )\n      r4_overlap_age = frame - self.v33r4_decel_entry_frame\n      r4_overlap_rising = (\n        r4_feedback[\"torque_actual\"] >\n        max(80, self.v33r4_decel_entry_torque + V33R4_ENTRY_TORQUE_RISE_RAW)\n      )\n      r4_overlap_unsafe = (\n        r4_positive_under_friction and\n        (\n          not r4_negative_intent or\n          self.v33r4_decel_torque_cleared or\n          r4_overlap_age > V33R4_ENTRY_OVERLAP_FRAMES or\n          r4_overlap_rising\n        )\n      )\n      if r4_overlap_unsafe:\n        self.v33r4_positive_overlap_counter = min(\n          self.v33r4_positive_overlap_counter + 1,\n          V33R4_OVERLAP_FAULT_FRAMES,\n        )\n      else:\n        self.v33r4_positive_overlap_counter = 0\n      if (\n        self.v33r4_positive_overlap_counter >=\n        V33R4_OVERLAP_FAULT_FRAMES\n      ):\n        self._v33r4_latch_fault(CS, \"positive_torque_under_friction_braking\")\n        control_allowed = False\n\n""",
"""      # V4.0 stock-like overlap supervision. The 2026-08-29 road logs proved\n      # that planner/0x273 decel intent can precede physical friction by\n      # 0.5-0.7 s. Starting the 0.55 s envelope from planner intent consumed the\n      # whole allowance before friction existed and falsely killed both long and\n      # lateral. Keep decel-direction tracking, but start the overlap clock only\n      # on the first ACTUAL friction + positive-torque observation.\n      r4_negative_intent = (\n        control_allowed and\n        (\n          hydraulic_req or\n          release_pump_active or\n          self.v33r2_decel_latched or\n          (plan_fresh and planner_brake_request >= V33R2_DECEL_LATCH_BRAKE)\n        )\n      )\n      if r4_negative_intent and self.v33r4_decel_entry_frame < 0:\n        self.v33r4_decel_entry_frame = frame\n        self.v33r4_decel_entry_torque = r4_feedback[\"torque_actual\"]\n        self.v33r4_decel_torque_cleared = (\n          r4_feedback[\"torque_actual\"] <= 80\n        )\n      if r4_negative_intent and r4_feedback[\"torque_actual\"] <= 80:\n        self.v33r4_decel_torque_cleared = True\n\n      r4_positive_under_friction = (\n        control_allowed and\n        r4_feedback_clean and\n        r4_feedback[\"friction\"] > 0 and\n        r4_feedback[\"positive_vote\"]\n      )\n\n      if r4_positive_under_friction and self.v40_overlap_start_frame < 0:\n        self.v40_overlap_start_frame = frame\n        self.v40_overlap_start_torque = r4_feedback[\"torque_actual\"]\n      elif (\n        not control_allowed or\n        not r4_negative_intent or\n        r4_feedback[\"torque_actual\"] <= 80\n      ):\n        self.v40_overlap_start_frame = -1000000\n        self.v40_overlap_start_torque = 0\n\n      v40_overlap_started = self.v40_overlap_start_frame >= 0\n      v40_overlap_age = (\n        frame - self.v40_overlap_start_frame\n        if v40_overlap_started else 0\n      )\n      v40_overlap_rising = (\n        v40_overlap_started and\n        r4_feedback[\"torque_actual\"] >\n        max(80, self.v40_overlap_start_torque + V33R4_ENTRY_TORQUE_RISE_RAW)\n      )\n      r4_overlap_unsafe = (\n        r4_positive_under_friction and\n        (\n          not r4_negative_intent or\n          self.v33r4_decel_torque_cleared or\n          (v40_overlap_started and\n           v40_overlap_age > V33R4_ENTRY_OVERLAP_FRAMES) or\n          v40_overlap_rising\n        )\n      )\n      if r4_overlap_unsafe:\n        self.v33r4_positive_overlap_counter = min(\n          self.v33r4_positive_overlap_counter + 1,\n          V33R4_OVERLAP_FAULT_FRAMES,\n        )\n      else:\n        self.v33r4_positive_overlap_counter = 0\n      if (\n        self.v33r4_positive_overlap_counter >=\n        V33R4_OVERLAP_FAULT_FRAMES\n      ):\n        self._v33r4_latch_fault(CS, \"positive_torque_under_friction_braking\")\n        control_allowed = False\n\n"""
),
(
"""      # Longitudinal engagement follows independent physical/cruise override\n      # state, not the stale outer `enabled` bit. This makes gas, brake,\n      # CANCEL, and cruise-latch loss encode an exact disabled command.\n      longitudinal_enabled = control_allowed\n      if not longitudinal_enabled:\n        brake_state = 0x00\n        pump_reaction = 0.0\n        brake_mag = 200\n        des_speed = 0.0\n      elif self.v25o_stop_hold or sng_release_active:\n""",
"""      # V4.0 keeps the ACC session visible/engaged during driver gas override\n      # just like stock, while `control_allowed` above still prohibits any OP\n      # brake or propulsion actuation. Brake/CANCEL/fault still encode disabled.\n      longitudinal_enabled = (\n        v40_session_allowed and not self.v33r4_fault_latched\n      )\n      if not longitudinal_enabled:\n        brake_state = 0x00\n        pump_reaction = 0.0\n        brake_mag = 200\n        des_speed = 0.0\n      elif v40_gas_override:\n        brake_state = 0x01\n        pump_reaction = 0.0\n        brake_mag = 200\n        des_speed = CS.out.vEgo\n      elif self.v25o_stop_hold or sng_release_active:\n"""
),
(
"""      if not longitudinal_enabled:\n        acc_cmd_is_accel = False\n        acc_cmd_is_decel = False\n      elif brake_state == 0x30:\n""",
"""      if not longitudinal_enabled:\n        acc_cmd_is_accel = False\n        acc_cmd_is_decel = False\n      elif v40_gas_override:\n        # Stock accepts SET under accelerator override and keeps 0x273 in the\n        # enabled/normal state. Hold ACC_CMD at current speed so the override\n        # cannot create hidden OP propulsion while the pedal is down.\n        acc_cmd_is_accel = True\n        acc_cmd_is_decel = False\n      elif brake_state == 0x30:\n"""
),
]

for i, (old, new) in enumerate(cc_replacements, 1):
    count = cc.count(old)
    if count != 1:
        raise SystemExit(f"carcontroller replacement {i}: expected 1 match, found {count}")
    cc = cc.replace(old, new, 1)

cs_replacements = [
(
"""    self.is_plus_btn_latch = False\n    self.is_minus_btn_latch = False\n    self.prev_distance_btn = False\n""",
"""    self.is_plus_btn_latch = False\n    self.is_minus_btn_latch = False\n    # V4.0 exposes a one-cycle SET/RES release edge to CarController so a\n    # longitudinal-only feedback fault can re-arm without dropping lateral.\n    self.v40_acc_rearm_edge = False\n    self.prev_distance_btn = False\n"""
),
(
"""    minus_button = bool(cp.vl[\"PCM_BUTTONS\"][\"SET_MINUS\"])\n    plus_button = bool(cp.vl[\"PCM_BUTTONS\"][\"RES_PLUS\"])\n\n    if self.is_cruise_latch:\n""",
"""    minus_button = bool(cp.vl[\"PCM_BUTTONS\"][\"SET_MINUS\"])\n    plus_button = bool(cp.vl[\"PCM_BUTTONS\"][\"RES_PLUS\"])\n\n    # V4.0: release edge of either physical SET/RES button. The previous latch\n    # values are updated at the end of this button block, so this is true for\n    # exactly one CarState cycle after the driver releases the button.\n    self.v40_acc_rearm_edge = bool(\n      (self.is_plus_btn_latch and not plus_button) or\n      (self.is_minus_btn_latch and not minus_button)\n    )\n\n    if self.is_cruise_latch:\n"""
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
