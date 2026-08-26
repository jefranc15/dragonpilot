#!/usr/bin/env python3
from pathlib import Path
import py_compile

PATH = Path("selfdrive/car/dnga/carcontroller.py")
old_text = PATH.read_text()
new_text = old_text

replacements = [
(
"""      decel_latch_request = (
        control_allowed and
        (
          hydraulic_req or
          sng_release_active or
          release_pump_active or
          (
            plan_fresh and
            planner_brake_request >= V33R2_DECEL_LATCH_BRAKE
          ) or
          self.v25l_speed_offset < -V25V_REGEN_OFFSET_EPS
        )
      )
""",
"""      # R4.1: the stock 1.2 s FC/04/C8 stage is protocol framing, not proof
      # that physical braking is still active. start_v33r_staged_release()
      # already owns the persistent latch, so the timed release-pump stage must
      # not continuously re-request/reset that latch.
      decel_latch_request = (
        control_allowed and
        (
          hydraulic_req or
          sng_release_active or
          (
            plan_fresh and
            planner_brake_request >= V33R2_DECEL_LATCH_BRAKE
          ) or
          self.v25l_speed_offset < -V25V_REGEN_OFFSET_EPS
        )
      )
"""
),
(
"""          not self.v25o_stop_hold and
          not hydraulic_req and
          not sng_release_active and
          not release_pump_active and
          r4_feedback_clean and
          r4_feedback[\"brakes_clear\"] and
""",
"""          not self.v25o_stop_hold and
          not hydraulic_req and
          not sng_release_active and
          r4_feedback_clean and
          r4_feedback[\"brakes_clear\"] and
"""
),
(
"""      target_slope_lock = (
        hydraulic_req or
        release_pump_active or
        self.v33r2_decel_latched or
""",
"""      target_slope_lock = (
        hydraulic_req or
        self.v33r2_decel_latched or
"""
),
(
"""      r4_torque_ready_candidate = (
        r4_accel_arm_ready and
        r4_feedback[\"torque_ramp_ready\"] and
        not hydraulic_req and
        not release_pump_active and
        not self.v33r2_decel_latched and
        positive_agreement
      )
""",
"""      r4_torque_ready_candidate = (
        r4_accel_arm_ready and
        r4_feedback[\"torque_ramp_ready\"] and
        not hydraulic_req and
        not self.v33r2_decel_latched and
        positive_agreement
      )
"""
),
(
"""      propulsion_blocked = (
        hydraulic_req or
        release_pump_active or
        self.v33r2_decel_latched or
""",
"""      # The 1.2 s FC/04/C8 release frame may coexist with positive hybrid
      # torque in stock. Physical feedback/latch/torque readiness, not the
      # protocol timer itself, decides whether propulsion may ramp.
      propulsion_blocked = (
        hydraulic_req or
        self.v33r2_decel_latched or
"""
),
(
"""      # Preserve the existing post-deceleration dwell, now driven by the
      # persistent latch and verified pump-release stage instead of a negative
      # 0x273 target. Low speed retains an additional neutral wake delay.
      regen_or_brake_active = (
        hydraulic_req or
        sng_release_active or
        release_pump_active or
        self.v33r2_decel_latched
      )
""",
"""      # R4.1: only physical/latched deceleration extends the neutral dwell.
      # The stock 1.2 s FC/04/C8 protocol stage can continue after the hybrid
      # system has already crossed through neutral into positive torque.
      regen_or_brake_active = (
        hydraulic_req or
        sng_release_active or
        self.v33r2_decel_latched
      )
"""
),
(
"""      low_speed_propulsion_request = (
        control_allowed and
        not hydraulic_req and
        not sng_release_active and
        not release_pump_active and
        not self.v33r2_decel_latched and
""",
"""      low_speed_propulsion_request = (
        control_allowed and
        not hydraulic_req and
        not sng_release_active and
        not self.v33r2_decel_latched and
"""
),
(
"""      elif (
        hydraulic_req or
        sng_release_active or
        release_pump_active or
        self.v33r2_decel_latched
      ):
        self.v25l_speed_offset = 0.0
""",
"""      elif (
        hydraulic_req or
        sng_release_active or
        self.v33r2_decel_latched
      ):
        self.v25l_speed_offset = 0.0
"""
),
(
"""      elif (
        hydraulic_req or
        release_pump_active or
        self.v33r2_decel_latched
      ):
        # Never combine braking intent with a lowered or positive 0x273 target.
        des_speed = CS.out.vEgo
""",
"""      elif (
        hydraulic_req or
        self.v33r2_decel_latched
      ):
        # Never combine physical/latched braking intent with a positive target.
        # A timed FC/04/C8 release frame alone is not physical braking.
        des_speed = CS.out.vEgo
"""
),
(
"""      elif (
        brake_state in (0x21, 0x31) or
        sng_release_active or
        release_pump_active or
        self.v33r2_decel_latched or
        low_speed_handoff_blocked or
        not r4_accel_arm_ready or
        not plan_fresh
      ):
        acc_cmd_is_accel = False
        acc_cmd_is_decel = True
      else:
""",
"""      elif (
        release_pump_active and
        not self.v33r2_decel_latched and
        r4_accel_arm_ready and
        plan_fresh
      ):
        # Stock 2026-08-26 capture: 0x271 stayed 0x01 + FC/04/C8 for 1.20 s,
        # while 0x273 changed 0x20 -> 0x40 and hybrid torque crossed positive.
        # Arm normal/ACCEL mode here, but target buildup remains independently
        # blocked by torque-ready feedback and the retained dwell/ramp gates.
        acc_cmd_is_accel = True
        acc_cmd_is_decel = False
      elif (
        brake_state in (0x21, 0x31) or
        sng_release_active or
        self.v33r2_decel_latched or
        low_speed_handoff_blocked or
        not r4_accel_arm_ready or
        not plan_fresh
      ):
        acc_cmd_is_accel = False
        acc_cmd_is_decel = True
      else:
"""
),
(
"""      low_speed_accel_unlock = (
        longitudinal_enabled and
        low_speed_propulsion_request and
        not hydraulic_req and
        not sng_release_active and
        not release_pump_active and
        not self.v33r2_decel_latched and
""",
"""      low_speed_accel_unlock = (
        longitudinal_enabled and
        low_speed_propulsion_request and
        not hydraulic_req and
        not sng_release_active and
        not self.v33r2_decel_latched and
"""
),
(
"""        # Copy the stock-observed moving release sequence. Keep the pump
        # reaction at FC/04/C8 and retain deceleration mode for the full
        # observed 1.2-second pressure-release interval.
""",
"""        # Copy the stock-observed moving release sequence. Keep FC/04/C8 for
        # the full observed 1.2-second protocol interval. R4.1 no longer treats
        # that timer alone as physical braking: 0x273 may arm normal mode and,
        # after verified neutral torque, ramp propulsion while FC/04/C8 remains.
"""
),
]

for i, (old, new) in enumerate(replacements, 1):
    count = new_text.count(old)
    if count != 1:
        raise SystemExit(f"replacement {i}: expected 1 match, found {count}")
    new_text = new_text.replace(old, new, 1)

PATH.write_text(new_text)
py_compile.compile(str(PATH), doraise=True)
print("R4.1 patch applied and py_compile passed")
