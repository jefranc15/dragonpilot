#!/usr/bin/env python3
from pathlib import Path
import py_compile

PATH = Path("selfdrive/car/dnga/carcontroller.py")
old_text = PATH.read_text()
new_text = old_text

replacements = [
(
'''      base_control_allowed = (\n        enabled and\n        CS.out.cruiseState.enabled and\n        not pcm_cancel_cmd and\n        not CS.out.gasPressed and\n        not CS.out.brakePressed\n      )\n      r4_feedback = hybrid_feedback_snapshot(CS, frame)\n      r4_feedback_clean = (\n        r4_feedback["fresh"] and r4_feedback["consistent"]\n      )\n      r4_rearm_ok = (\n        r4_feedback_clean and\n        r4_feedback["brakes_clear"] and\n        r4_feedback["torque_ramp_ready"] and\n        not r4_feedback["positive_vote"]\n      )\n''',
'''      # R4.2 separates the ACC session from actuator authority. Stock accepts\n      # SET/RES and shows the set speed while the driver is overriding with the\n      # accelerator; only brake/CANCEL/session loss disable the 0x271/0x273\n      # session. Gas still blocks every OP brake/propulsion actuator below.\n      base_session_allowed = (\n        enabled and\n        CS.out.cruiseState.enabled and\n        not pcm_cancel_cmd and\n        not CS.out.brakePressed\n      )\n      base_control_allowed = (\n        base_session_allowed and\n        not CS.out.gasPressed\n      )\n      r4_feedback = hybrid_feedback_snapshot(CS, frame)\n      r4_feedback_clean = (\n        r4_feedback["fresh"] and r4_feedback["consistent"]\n      )\n      # Rearm is a session-level decision, not permission for propulsion. A\n      # clean, brake-clear hybrid state may rearm even while torque is not yet\n      # neutral; r4_torque_ready_candidate remains the independent propulsion\n      # gate after the accelerator is released.\n      r4_rearm_ok = (\n        r4_feedback_clean and\n        r4_feedback["brakes_clear"]\n      )\n'''
),
(
'''      control_allowed = (\n        base_control_allowed and not self.v33r4_fault_latched\n      )\n      CS.hybrid_feedback_fault = self.v33r4_fault_latched\n''',
'''      control_allowed = (\n        base_control_allowed and not self.v33r4_fault_latched\n      )\n      longitudinal_session_allowed = (\n        base_session_allowed and not self.v33r4_fault_latched\n      )\n      gas_override_active = (\n        longitudinal_session_allowed and CS.out.gasPressed\n      )\n      CS.hybrid_feedback_fault = self.v33r4_fault_latched\n'''
),
(
'''      # Toyota briefly overlaps positive hybrid torque with brake entry. In\n      # the passive stock capture the longest voted positive-torque + friction\n      # overlap was 0.20 s. Permit a wider 0.55 s entry envelope only while\n      # torque does not rise materially; once torque has cleared, any return of\n      # voted propulsion under friction is a fault.\n      r4_negative_intent = (\n        control_allowed and\n        (\n          hydraulic_req or\n          release_pump_active or\n          self.v33r2_decel_latched or\n          (plan_fresh and planner_brake_request >= V33R2_DECEL_LATCH_BRAKE)\n        )\n      )\n      if r4_negative_intent and self.v33r4_decel_entry_frame < 0:\n        self.v33r4_decel_entry_frame = frame\n        self.v33r4_decel_entry_torque = r4_feedback["torque_actual"]\n        self.v33r4_decel_torque_cleared = (\n          r4_feedback["torque_actual"] <= 80\n        )\n      if r4_negative_intent and r4_feedback["torque_actual"] <= 80:\n        self.v33r4_decel_torque_cleared = True\n\n      r4_positive_under_friction = (\n        control_allowed and\n        r4_feedback_clean and\n        r4_feedback["friction"] > 0 and\n        r4_feedback["positive_vote"]\n      )\n      r4_overlap_age = frame - self.v33r4_decel_entry_frame\n      r4_overlap_rising = (\n        r4_feedback["torque_actual"] >\n        max(80, self.v33r4_decel_entry_torque + V33R4_ENTRY_TORQUE_RISE_RAW)\n      )\n      r4_overlap_unsafe = (\n        r4_positive_under_friction and\n        (\n          not r4_negative_intent or\n          self.v33r4_decel_torque_cleared or\n          r4_overlap_age > V33R4_ENTRY_OVERLAP_FRAMES or\n          r4_overlap_rising\n        )\n      )\n''',
'''      # Toyota briefly overlaps positive hybrid torque with physical friction\n      # brake entry. The 2026-08-29 R4.1 logs showed the old timer started\n      # 0.5-0.7 s too early from planner/DECEL intent, exhausting the entire\n      # allowance before friction even appeared and falsely dropping control.\n      # R4.2 starts the 0.55 s envelope only when the actual measured overlap\n      # (friction > 0 + positive torque vote) begins. Rising torque, persistence\n      # past the envelope, or positive torque returning after neutral remains a\n      # fault.\n      r4_negative_intent = (\n        control_allowed and\n        (\n          hydraulic_req or\n          release_pump_active or\n          self.v33r2_decel_latched or\n          (plan_fresh and planner_brake_request >= V33R2_DECEL_LATCH_BRAKE)\n        )\n      )\n      r4_positive_under_friction = (\n        control_allowed and\n        r4_feedback_clean and\n        r4_feedback["friction"] > 0 and\n        r4_feedback["positive_vote"]\n      )\n\n      if not r4_negative_intent:\n        self.v33r4_decel_entry_frame = -1000000\n        self.v33r4_decel_entry_torque = 0\n        self.v33r4_decel_torque_cleared = False\n      elif r4_feedback_clean and r4_feedback["torque_actual"] <= 80:\n        # Once the powertrain has crossed through the positive-torque region,\n        # any later return to voted propulsion under friction is not an entry\n        # transient and should fault immediately.\n        self.v33r4_decel_torque_cleared = True\n\n      if (\n        r4_positive_under_friction and\n        self.v33r4_decel_entry_frame < 0\n      ):\n        self.v33r4_decel_entry_frame = frame\n        self.v33r4_decel_entry_torque = r4_feedback["torque_actual"]\n\n      r4_overlap_started = self.v33r4_decel_entry_frame >= 0\n      r4_overlap_age = (\n        frame - self.v33r4_decel_entry_frame\n        if r4_overlap_started else 0\n      )\n      r4_overlap_rising = (\n        r4_positive_under_friction and\n        r4_overlap_started and\n        r4_feedback["torque_actual"] >\n        max(80, self.v33r4_decel_entry_torque + V33R4_ENTRY_TORQUE_RISE_RAW)\n      )\n      r4_overlap_unsafe = (\n        r4_positive_under_friction and\n        (\n          not r4_negative_intent or\n          self.v33r4_decel_torque_cleared or\n          (\n            r4_overlap_started and\n            r4_overlap_age > V33R4_ENTRY_OVERLAP_FRAMES\n          ) or\n          r4_overlap_rising\n        )\n      )\n'''
),
(
'''      # Longitudinal engagement follows independent physical/cruise override\n      # state, not the stale outer `enabled` bit. This makes gas, brake,\n      # CANCEL, and cruise-latch loss encode an exact disabled command.\n      longitudinal_enabled = control_allowed\n      if not longitudinal_enabled:\n        brake_state = 0x00\n        pump_reaction = 0.0\n        brake_mag = 200\n        des_speed = 0.0\n      elif self.v25o_stop_hold or sng_release_active:\n''',
'''      # Keep the SET/RES ACC session visible through a driver accelerator\n      # override, matching the stock camera. Actuator authority remains\n      # `control_allowed`, so gas cannot produce OP braking or propulsion.\n      longitudinal_enabled = longitudinal_session_allowed\n      if not longitudinal_enabled:\n        brake_state = 0x00\n        pump_reaction = 0.0\n        brake_mag = 200\n        des_speed = 0.0\n      elif gas_override_active:\n        # Stock-like neutral override framing: preserve enabled 0x01/0x273 and\n        # the cluster set speed, but request no OP acceleration/deceleration.\n        brake_state = 0x01\n        pump_reaction = 0.0\n        brake_mag = 200\n        des_speed = CS.out.vEgo\n      elif self.v25o_stop_hold or sng_release_active:\n'''
),
(
'''      if not longitudinal_enabled:\n        acc_cmd_is_accel = False\n        acc_cmd_is_decel = False\n      elif brake_state == 0x30:\n''',
'''      if not longitudinal_enabled:\n        acc_cmd_is_accel = False\n        acc_cmd_is_decel = False\n      elif gas_override_active:\n        # Stock accepts SET while the driver holds the accelerator. Keep the\n        # normal/ACCEL mode bit armed with an exact current-speed target; OP\n        # positive-target authority remains blocked until gas is released and\n        # the R4 torque-ready gate passes.\n        acc_cmd_is_accel = True\n        acc_cmd_is_decel = False\n      elif brake_state == 0x30:\n'''
),
(
'''          longitudinal_enabled,  # Independently override-gated longitudinal state\n''',
'''          longitudinal_enabled,  # SET/RES session state; gas override stays enabled\n'''
),
]

for i, (old, new) in enumerate(replacements, 1):
    count = new_text.count(old)
    if count != 1:
        raise SystemExit(f"replacement {i}: expected 1 match, found {count}")
    new_text = new_text.replace(old, new, 1)

if new_text == old_text:
    raise SystemExit("no changes generated")

PATH.write_text(new_text)
py_compile.compile(str(PATH), doraise=True)

# Static invariants for the three R4.2 fixes.
assert 'longitudinal_enabled = longitudinal_session_allowed' in new_text
assert 'gas_override_active' in new_text
assert 'r4_rearm_ok = (\n        r4_feedback_clean and\n        r4_feedback["brakes_clear"]\n      )' in new_text
assert 'r4_positive_under_friction and\n        self.v33r4_decel_entry_frame < 0' in new_text
print("R4.2 engagement/fault patch applied and py_compile passed")
