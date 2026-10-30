"""DNGA V4.3.10 coast-first longitudinal policy.

V4.3.10 removes deliberate below-vEgo 0x273 braking targets. Mild planner
deceleration is handled as a neutral coast, while confirmed braking is owned by
0x271 hydraulic control. Brake-specific 0x275 word-0 plus 0x08C friction remain
the hard propulsion veto during brake-to-accel handoff.

The raw torque-related 0x275/0x2C9/0x12A/0x125 values remain logging-only.
"""

from common.numpy_fast import clip
from selfdrive.car.dnga.dnga_hybrid_feedback import hybrid_brake_feedback_state
from selfdrive.car.dnga.longitudinal_op_only import LongitudinalController as OpenpilotOnlyLongitudinalController
from selfdrive.car.dnga.values import BrakeState, CarControllerParams, LongitudinalParams as P


COAST_RELEASE_PUMP_FRAMES = 80  # 0.80 s at 100 Hz; physical brake feedback still owns accel veto
KINEMATIC_READY_AEGO = 0.35
KINEMATIC_SURGE_AEGO = 0.30
KINEMATIC_READY_COUNT = 2  # ~0.10 s at the 20 Hz DNGA ACC update rate


class LongitudinalController(OpenpilotOnlyLongitudinalController):
  def __init__(self):
    super().__init__()
    self.kinematic_ready_counter = 0
    self.kinematic_last_a_ego = 0.0

  def _update_near_set_coast(self, CS, plan, lead_state, apply_accel):
    # Never synthesize a negative 0x273 target near the cruise setpoint.
    self.near_set_coast_active = False
    self.near_set_coast_counter = 0
    return False

  @staticmethod
  def _hybrid_brake_state(session):
    # Only 0x275 brake request and 0x08C friction have control authority.
    return hybrid_brake_feedback_state(session.feedback)

  def _start_staged_release(self, frame):
    """Use a shorter neutral release phase; feedback still blocks early accel."""
    self.handoff_pending = True
    self.handoff_ready_counter = 0
    self.kinematic_ready_counter = 0
    self.release_pump_until_frame = max(
      self.release_pump_until_frame, frame + COAST_RELEASE_PUMP_FRAMES
    )
    self.release_lead_until_frame = max(
      self.release_lead_until_frame, frame + P.RELEASE_LEAD_HOLD_FRAMES
    )
    self.speed_offset = 0.0

  def _update_hydraulic(self, CS, frame, session, apply_accel, plan, lead_state):
    """Do not let positive PID demand release braking against a negative plan."""
    hydraulic_accel = apply_accel
    if plan.fresh and plan.accel < -P.DECEL_DEADBAND and hydraulic_accel > 0.0:
      hydraulic_accel = 0.0
    return super()._update_hydraulic(
      CS, frame, session, hydraulic_accel, plan, lead_state
    )

  def _update_handoff_feedback(self, session, propulsion, brake):
    """Gate propulsion after real braking, but do not treat neutral coast as brake."""
    brake_feedback_fresh, physical_braking = self._hybrid_brake_state(session)
    release_active = brake.hydraulic or brake.sng_release or brake.release_pump
    unexpected_surge = (
      self.kinematic_last_a_ego >= KINEMATIC_SURGE_AEGO
      and not propulsion.positive_agreement
    )

    # Mild negative planner demand is COAST, not a brake event. Only commanded
    # 0x271/release activity, physical brake feedback, or an unexplained surge
    # arms the brake-to-propulsion handoff.
    if release_active or physical_braking or unexpected_surge:
      self.handoff_pending = True
      self.handoff_ready_counter = 0
      self.kinematic_ready_counter = 0

    if not session.enabled:
      self.handoff_pending = False
      self.handoff_ready_counter = 0
      self.kinematic_ready_counter = 0
      propulsion.ramp_ready = True
      return

    # Once actual braking has occurred, require fresh/clear physical feedback
    # and explicit planner+PID positive agreement. A modest positive-aEgo cap
    # blocks a jump-forward handoff but is much less restrictive than V4.3.9.
    ready_candidate = (
      self.handoff_pending
      and not release_active
      and brake_feedback_fresh
      and not physical_braking
      and propulsion.positive_agreement
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
    """Keep 0x273 neutral for coast/brake; only positive propulsion gets offset."""
    if not session.allowed:
      self.speed_offset = 0.0
      if not session.enabled:
        self.handoff_pending = False
        self.handoff_ready_counter = 0
        self.kinematic_ready_counter = 0
      return

    brake_feedback_fresh, physical_braking = self._hybrid_brake_state(session)

    # 0x271 owns actual braking. 0x273 stays exactly at vEgo, so it does not
    # request extra regen/engine braking on top of the hydraulic command.
    if brake.hydraulic or brake.release_pump or brake.sng_release:
      self.speed_offset = 0.0
      self.handoff_pending = True
      self.handoff_ready_counter = 0
      self.kinematic_ready_counter = 0
      return

    # If the HEV is still physically braking after 0x271 releases, remain
    # neutral and wait. Do not deepen the speed target or arm propulsion.
    if brake_feedback_fresh and physical_braking:
      self.speed_offset = 0.0
      self.handoff_pending = True
      self.handoff_ready_counter = 0
      self.kinematic_ready_counter = 0
      return

    # Mild planner deceleration is a true coast state: no hydraulic brake and
    # no deliberate negative ACC_CMD offset.
    if propulsion.accel <= -P.DECEL_DEADBAND:
      self.speed_offset = 0.0
      return

    # After a real brake event, stay neutral until the handoff gate clears.
    if self.handoff_pending:
      self.speed_offset = 0.0
      return

    # Positive propulsion keeps the existing bounded/ramped V4.3 behavior.
    return super()._update_speed_offset(CS, frame, session, propulsion, plan, brake)

  def _build_command(self, CS, frame, session, plan, lead_state, brake, propulsion):
    command = super()._build_command(CS, frame, session, plan, lead_state, brake, propulsion)

    if not session.enabled or session.gas_override or brake.state == BrakeState.HOLD:
      return command

    brake_feedback_fresh, physical_braking = self._hybrid_brake_state(session)

    # Actual commanded braking keeps DECEL framing, but ACC_CMD is neutral.
    if brake.hydraulic or brake.sng_release or brake.release_pump:
      command.speed = 0.0 if brake.sng_release else float(CS.out.vEgo)
      command.is_accel = False
      command.is_decel = True
      return command

    # Physical brake/regen can linger after 0x271 has released. The accel veto
    # remains hard, but use neutral framing to let the HEV unwind instead of
    # perpetuating regen/engine braking through IS_DECEL.
    if brake_feedback_fresh and physical_braking:
      command.speed = float(CS.out.vEgo)
      command.is_accel = False
      command.is_decel = False
      return command

    # Mild slowing and post-brake handoff are true coast: both mode bits zero.
    if propulsion.accel <= -P.DECEL_DEADBAND or self.handoff_pending:
      command.speed = float(CS.out.vEgo)
      command.is_accel = False
      command.is_decel = False
      return command

    return command

  def update(self, enabled, CS, frame, accel, pcm_cancel_cmd, lead):
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
