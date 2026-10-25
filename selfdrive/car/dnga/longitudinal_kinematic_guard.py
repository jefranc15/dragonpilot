"""DNGA V4.3.9 planner/kinematic longitudinal guard.

V4.3.8 proved that the raw torque-related 0x275/0x2C9/0x12A/0x125 values are
not validated well enough to close the longitudinal loop around them. V4.3.9
keeps only the brake-specific 0x275 word-0 request plus 0x08C friction as a
physical-braking veto. Those signals can block propulsion handoff, but they
never hold hydraulic BRAKE_MIN and never deepen the 0x273 target.
"""

from common.numpy_fast import clip, interp
from selfdrive.car.dnga.dnga_hybrid_feedback import hybrid_brake_feedback_state
from selfdrive.car.dnga.longitudinal_op_only import LongitudinalController as OpenpilotOnlyLongitudinalController
from selfdrive.car.dnga.values import BrakeState, CarControllerParams, LongitudinalParams as P


# Keep the 0x273 decel target modest. This is intentionally far below the
# ~5 km/h separation reached by V4.3.8 during the reproduced rev event.
KINEMATIC_DECEL_CAP_BP = [0.0, 5.0, 15.0, 25.0, 35.0]
KINEMATIC_DECEL_CAP_V = [0.05, 0.08, 0.16, 0.24, 0.30]  # m/s below vEgo
KINEMATIC_READY_AEGO = 0.15
KINEMATIC_SURGE_AEGO = 0.25
KINEMATIC_READY_COUNT = 4  # ~0.20 s at the 20 Hz DNGA ACC update rate


class LongitudinalController(OpenpilotOnlyLongitudinalController):
  def __init__(self):
    super().__init__()
    self.kinematic_ready_counter = 0
    self.kinematic_last_a_ego = 0.0

  def _update_near_set_coast(self, CS, plan, lead_state, apply_accel):
    # Do not synthesize no-lead decel on this HEV. The V4.3.5 logs showed that
    # even a tiny extra negative target can select disproportionate powertrain
    # braking/engine behavior.
    self.near_set_coast_active = False
    self.near_set_coast_counter = 0
    return False

  @staticmethod
  def _hybrid_brake_state(session):
    # This helper intentionally inspects only 0x275 brake request and 0x08C
    # friction. Torque-request/actual channels have no control authority.
    return hybrid_brake_feedback_state(session.feedback)

  @staticmethod
  def _kinematic_decel_cap(v_ego):
    return float(interp(v_ego, KINEMATIC_DECEL_CAP_BP, KINEMATIC_DECEL_CAP_V))

  def _update_handoff_feedback(self, session, propulsion, brake):
    """Gate propulsion with commanded state, physical brake state, and aEgo."""
    brake_feedback_fresh, physical_braking = self._hybrid_brake_state(session)
    release_active = brake.hydraulic or brake.sng_release or brake.release_pump
    decel_commanded = (
      release_active
      or physical_braking
      or propulsion.accel <= -P.DECEL_DEADBAND
      or self.speed_offset < -P.SPEED_OFFSET_EPS
    )
    unexpected_surge = (
      self.kinematic_last_a_ego >= KINEMATIC_SURGE_AEGO
      and not propulsion.positive_agreement
    )

    if decel_commanded or unexpected_surge:
      self.handoff_pending = True
      self.handoff_ready_counter = 0
      self.kinematic_ready_counter = 0

    if not session.enabled:
      self.handoff_pending = False
      self.handoff_ready_counter = 0
      self.kinematic_ready_counter = 0
      propulsion.ramp_ready = True
      return

    # After any commanded braking/release, propulsion cannot re-arm until the
    # brake-specific CAN feedback is fresh and clear. This preserves the old
    # anti-simultaneous-regen/accel protection without consulting torque data.
    ready_candidate = (
      self.handoff_pending
      and not release_active
      and brake_feedback_fresh
      and not physical_braking
      and propulsion.accel > -P.DECEL_DEADBAND
      and self.speed_offset >= -P.SPEED_OFFSET_EPS
      and self.kinematic_last_a_ego <= KINEMATIC_READY_AEGO
    )
    if ready_candidate:
      self.kinematic_ready_counter = min(
        self.kinematic_ready_counter + 1, KINEMATIC_READY_COUNT
      )
    elif self.handoff_pending:
      self.kinematic_ready_counter = 0

    if self.handoff_pending and self.kinematic_ready_counter >= KINEMATIC_READY_COUNT:
      self.handoff_pending = False
      self.handoff_ready_counter = 0
      self.kinematic_ready_counter = 0

    propulsion.ramp_ready = not self.handoff_pending

  def _update_speed_offset(self, CS, frame, session, propulsion, plan, brake):
    if not session.allowed:
      self.speed_offset = 0.0
      if not session.enabled:
        self.handoff_pending = False
        self.handoff_ready_counter = 0
        self.kinematic_ready_counter = 0
      return

    if brake.release_pump or brake.sng_release:
      self.speed_offset = 0.0
      self.handoff_pending = True
      self.handoff_ready_counter = 0
      self.kinematic_ready_counter = 0
      return

    _, physical_braking = self._hybrid_brake_state(session)
    if physical_braking and not brake.hydraulic:
      # The HEV is still reporting regen/brake activity after our hydraulic
      # command has ended. Hold a neutral target and DECEL framing; do not add
      # more negative target and do not permit propulsion yet.
      self.speed_offset = 0.0
      self.handoff_pending = True
      self.handoff_ready_counter = 0
      self.kinematic_ready_counter = 0
      return

    if brake.hydraulic or propulsion.accel <= -P.DECEL_DEADBAND:
      max_offset = self._kinematic_decel_cap(float(CS.out.vEgo))
      t_lookup = 0.35 + 0.07 * float(CS.out.vEgo)
      requested_decel = max(0.0, -float(propulsion.accel))
      target_offset = -min(max_offset, requested_decel * t_lookup)

      current_offset = float(clip(self.speed_offset, -max_offset, 0.0))
      if target_offset < current_offset:
        current_offset = max(target_offset, current_offset - P.DECEL_OFFSET_STEP_DOWN)
      else:
        current_offset = min(target_offset, current_offset + P.DECEL_OFFSET_STEP_UP)
      self.speed_offset = current_offset
      self.handoff_pending = True
      self.handoff_ready_counter = 0
      self.kinematic_ready_counter = 0
      return

    # Never carry a negative target into propulsion. Return to vEgo first.
    if self.speed_offset < -P.SPEED_OFFSET_EPS:
      self.speed_offset = min(0.0, self.speed_offset + P.TARGET_RETURN_STEP)
      self.handoff_pending = True
      self.handoff_ready_counter = 0
      self.kinematic_ready_counter = 0
      return
    if self.speed_offset < 0.0:
      self.speed_offset = 0.0

    # While handoff is pending, hold a neutral desired speed. Do not build a
    # positive offset until physical braking has cleared and the car is calm.
    if self.handoff_pending:
      self.speed_offset = 0.0
      return

    return super()._update_speed_offset(CS, frame, session, propulsion, plan, brake)

  def _build_command(self, CS, frame, session, plan, lead_state, brake, propulsion):
    command = super()._build_command(CS, frame, session, plan, lead_state, brake, propulsion)

    if not session.enabled or session.gas_override or brake.state == BrakeState.HOLD:
      return command

    _, physical_braking = self._hybrid_brake_state(session)
    decel_mode = (
      brake.hydraulic
      or brake.sng_release
      or brake.release_pump
      or physical_braking
      or self.handoff_pending
      or self.speed_offset < -P.SPEED_OFFSET_EPS
      or propulsion.accel <= -P.DECEL_DEADBAND
    )
    if not decel_mode:
      return command

    if brake.sng_release:
      command.speed = 0.0
    else:
      command.speed = max(0.0, float(CS.out.vEgo) + min(0.0, float(self.speed_offset)))

    command.is_accel = False
    command.is_decel = True
    return command

  def update(self, enabled, CS, frame, accel, pcm_cancel_cmd, lead):
    # This mirrors the base update order while deliberately removing all
    # torque-based handoff logic. Brake-specific 0x275/0x08C feedback remains
    # available later in _update_handoff_feedback as a propulsion veto only.
    self.kinematic_last_a_ego = float(CS.out.aEgo)
    apply_accel = clip(accel, -3.0, 1.5)
    engagement_edge = enabled and not self.prev_enabled
    if engagement_edge:
      self._reset(frame, float(CS.out.aEgo))
      self.block_brake_until_frame = frame + 50
      self.low_speed_guard_until_frame = frame + P.LOW_SPEED_ENGAGEMENT_GUARD_FRAMES
      self.kinematic_ready_counter = 0
    self.prev_enabled = enabled

    if frame % CarControllerParams.ACC_STEP:
      return None

    self._update_messages(frame)
    plan = self._get_plan(frame, apply_accel)
    lead_state = self._update_lead(CS, frame, lead, plan)
    session = self._update_session(enabled, CS, frame, pcm_cancel_cmd)
    brake = self._update_hydraulic(CS, frame, session, apply_accel, plan, lead_state)
    self._encode_brake(CS, session.enabled, brake)
    return self._update_propulsion(CS, frame, session, apply_accel, plan, lead_state, brake)
