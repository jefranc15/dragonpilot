"""DNGA V5.0 coast-first longitudinal policy.

V5.0 keeps 0x273 neutral during coast and hydraulic braking, releases normal
moving 0x271 braking into coast when measured/PID demand shows the car is
already slowing enough, and removes the normal moving READY/pump release tail
that V4.3.10 logs showed could keep RPM rising after hydraulic release.

Brake-specific 0x275 word-0 plus 0x08C friction remain the hard propulsion
veto. Raw torque-related 0x275/0x2C9/0x12A/0x125 values remain logging-only.
"""

from dataclasses import replace

from common.numpy_fast import clip, interp
from selfdrive.car.dnga.dnga_hybrid_feedback import hybrid_brake_feedback_state
from selfdrive.car.dnga.longitudinal_op_only import LongitudinalController as OpenpilotOnlyLongitudinalController
from selfdrive.car.dnga.longitudinal_r1_policy import (
  CoastPreparationLatch, should_prepare_coast, should_abort_handoff,
)
from selfdrive.car.dnga.values import BrakeState, CarControllerParams, LongitudinalParams as P


# This branch carries executable candidates for off-car replay. Keep disabled
# for vehicle actuation until the full longitudinal controller and Panda safety
# have passed regression plus independent physical brake-response validation.
R1_EXPERIMENTAL_LONGITUDINAL_ENABLED = False

# V5.0 normal moving brake -> coast policy.
COAST_HANDOFF_MIN_SPEED = 10.0  # m/s; keep low-speed/SNG behavior conservative
COAST_HANDOFF_PID_ACCEL = 0.10  # downstream PID says braking is already excessive
COAST_HANDOFF_AEGO_MAX = -0.05  # car must still be measurably decelerating
COAST_HANDOFF_CONFIRM_COUNT = 4  # ~0.20 s at the 20 Hz DNGA ACC update rate
COAST_HANDOFF_BRAKE_BP = [10.0, 15.0, 25.0, 35.0]
COAST_HANDOFF_BRAKE_V = [0.18, 0.22, 0.26, 0.28]
COAST_REENTRY_FRAMES = 40  # 0.40 s normal re-entry hysteresis; urgent/stop paths bypass it

# Post-brake propulsion may arm only once the release surge/remaining decel is calm.
KINEMATIC_READY_AEGO_MIN = -0.20
KINEMATIC_READY_AEGO_MAX = 0.15
KINEMATIC_SURGE_AEGO = 0.25
KINEMATIC_READY_COUNT = 3  # ~0.15 s at the 20 Hz DNGA ACC update rate


class LongitudinalController(OpenpilotOnlyLongitudinalController):
  def __init__(self):
    super().__init__()
    self.kinematic_ready_counter = 0
    self.kinematic_last_a_ego = 0.0
    self.coast_release_counter = 0

  def _reset(self, frame, a_ego):
    super()._reset(frame, a_ego)
    self.r1_coast_prepare = CoastPreparationLatch()

  def _update_near_set_coast(self, CS, plan, lead_state, apply_accel):
    # Never synthesize a negative 0x273 target near the cruise setpoint.
    self.near_set_coast_active = False
    self.near_set_coast_counter = 0
    return False

  @staticmethod
  def _hybrid_brake_state(session):
    # Only 0x275 brake request and 0x08C friction have control authority.
    return hybrid_brake_feedback_state(session.feedback)

  @staticmethod
  def _coast_handoff_brake_limit(v_ego):
    return float(interp(v_ego, COAST_HANDOFF_BRAKE_BP, COAST_HANDOFF_BRAKE_V))

  def _start_staged_release(self, frame):
    """Release normal moving hydraulic braking directly into neutral coast.

    The inherited SNG release still emits its own brief READY/-0.4 sequence via
    brake.sng_release. What is removed here is the extra normal moving
    release_pump tail that V4.3.10 showed could keep RPM climbing for ~0.8 s.
    """
    self.handoff_pending = True
    self.handoff_ready_counter = 0
    self.kinematic_ready_counter = 0
    self.release_pump_until_frame = frame
    self.release_lead_until_frame = max(
      self.release_lead_until_frame, frame + P.RELEASE_LEAD_HOLD_FRAMES
    )
    self.speed_offset = 0.0

  def _clear_hydraulic(self, frame, reentry=True):
    super()._clear_hydraulic(frame, reentry=reentry)
    self.coast_release_counter = 0
    self.r1_coast_prepare.reset()
    if reentry:
      # Prevent the V4.3.9-style brake-off/brake-on chatter. The base urgent,
      # stopped-lead and guard entry paths explicitly bypass this deadline.
      self.brake_reentry_frame = max(
        self.brake_reentry_frame, frame + COAST_REENTRY_FRAMES
      )

  def _update_hydraulic(self, CS, frame, session, apply_accel, plan, lead_state):
    """Let sustained positive PID demand hand normal moving braking to coast.

    V4.3.10 suppressed positive PID while the planner was still negative. That
    stopped chatter, but the logs showed it could hold 0x271 for many seconds
    after the car was already slowing enough, with RPM building throughout.

    V5.0 admits a bounded coast handoff only at >=10 m/s, only with a trusted
    lead and fresh plan, only while the car is still decelerating, and never in
    urgent/emergency/SNG/hold contexts. The original plan remains authoritative
    everywhere else and immediately after this call.
    """
    if R1_EXPERIMENTAL_LONGITUDINAL_ENABLED:
      radar_age = frame - self.radar_frame
      radar_fresh = 0 <= radar_age <= 15

      # V5.0's inherited handoff_active latch can survive after positive PID
      # demand disappears. Do not let an OLD release request override NEW
      # negative PID + fresh planner braking demand. This is release-latch
      # cancellation only: no new brake entry or increased magnitude.
      if should_abort_handoff(
        allowed=session.allowed,
        braking=self.brake_active,
        curve=self.curve_brake_active,
        hold=self.stop_hold or self.sng_armed,
        plan_fresh=plan.fresh,
        lead_relevant=lead_state.relevant,
        lead_status=lead_state.status and radar_fresh,
        planner_brake=plan.brake,
        pid_accel=apply_accel,
      ):
        self.handoff_active = False
        self.handoff_counter = 0
        self.release_counter = 0

    emergency_closing = (
      lead_state.relevant
      and lead_state.status
      and (lead_state.closing_speed >= P.EMERGENCY_CLOSING_SPEED)
      and (lead_state.ttc <= P.EMERGENCY_TTC)
      and (plan.brake >= P.EMERGENCY_PLANNER_BRAKE)
    )
    urgent_closing = (
      (
        lead_state.urgent
        and (
          plan.brake >= P.URGENT_PLANNER_DECEL
          or (
            lead_state.closing_speed >= P.URGENT_CLOSING_SPEED
            and lead_state.ttc <= P.URGENT_TTC
          )
        )
      )
      or emergency_closing
    )

    if R1_EXPERIMENTAL_LONGITUDINAL_ENABLED:
      # Confirm a fast-closing, trusted lead for 3 x 20Hz cycles, then hold
      # 0x273 at vEgo with both mode bits zero. No negative speed target,
      # regen request, hydraulic threshold change or braking delay is added.
      coast_eligible = should_prepare_coast(
        allowed=session.allowed,
        moving_speed=float(CS.out.vEgo),
        plan_fresh=plan.fresh,
        lead_relevant=lead_state.relevant,
        lead_status=lead_state.status and radar_fresh,
        distance=lead_state.distance,
        closing=lead_state.closing_speed,
        ttc=lead_state.ttc,
        urgent=urgent_closing or self.urgent_brake,
        curve=plan.curve_active,
        hydraulic=self.brake_active,
        release=frame < self.release_pump_until_frame or self.sng_release_count > 0,
        hold=self.stop_hold or self.sng_armed,
      )
      self.r1_coast_prepare.update(coast_eligible)
    else:
      self.r1_coast_prepare.reset()

    coast_release_candidate = (
      self.brake_active
      and (not self.curve_brake_active)
      and session.allowed
      and (CS.out.vEgo >= COAST_HANDOFF_MIN_SPEED)
      and plan.fresh
      and lead_state.relevant
      and lead_state.status
      and (not urgent_closing)
      and (not self.urgent_brake)
      and (not self.sng_armed)
      and (not self.stop_hold)
      and (apply_accel >= COAST_HANDOFF_PID_ACCEL)
      and (CS.out.aEgo <= COAST_HANDOFF_AEGO_MAX)
      and (plan.brake <= self._coast_handoff_brake_limit(float(CS.out.vEgo)))
    )

    if coast_release_candidate:
      self.coast_release_counter = min(
        self.coast_release_counter + 1, COAST_HANDOFF_CONFIRM_COUNT
      )
    else:
      self.coast_release_counter = 0

    hydraulic_plan = plan
    if self.coast_release_counter >= COAST_HANDOFF_CONFIRM_COUNT:
      # Present only the hydraulic sub-state with a release-level plan so the
      # inherited rate-limited release machinery ramps 0x271 down smoothly.
      # _update_propulsion later receives the untouched original plan.
      hydraulic_plan = replace(
        plan,
        accel=max(float(plan.accel), -0.04),
        brake=min(float(plan.brake), max(0.0, P.PLANNER_RELEASE - 0.01)),
        lead_lookahead_brake=0.0,
      )

    brake = super()._update_hydraulic(
      CS, frame, session, apply_accel, hydraulic_plan, lead_state
    )
    if brake.hydraulic or brake.release_pump or brake.sng_release:
      self.r1_coast_prepare.reset()
    return brake

  def _update_handoff_feedback(self, session, propulsion, brake):
    """Gate propulsion after real braking until the HEV and chassis are calm."""
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

    # No fixed normal-moving dwell: once 0x271 is gone, acceleration can return
    # promptly if physical braking is fresh/clear and vehicle acceleration has
    # settled. The lower bound also prevents acceleration while still strongly
    # decelerating; the upper bound blocks the V4.3.10 release surge.
    ready_candidate = (
      self.handoff_pending
      and not release_active
      and brake_feedback_fresh
      and not physical_braking
      and propulsion.positive_agreement
      and self.speed_offset >= -P.SPEED_OFFSET_EPS
      and (
        KINEMATIC_READY_AEGO_MIN
        <= self.kinematic_last_a_ego
        <= KINEMATIC_READY_AEGO_MAX
      )
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
    if (R1_EXPERIMENTAL_LONGITUDINAL_ENABLED and self.r1_coast_prepare.active
        and session.allowed and not brake.hydraulic and not brake.release_pump
        and not brake.sng_release and not self.stop_hold):
      self.speed_offset = 0.0
      return
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

    # Actual commanded braking (and the explicit low-speed SNG release) keeps
    # DECEL framing, but ACC_CMD is neutral during moving hydraulic braking.
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

    if (R1_EXPERIMENTAL_LONGITUDINAL_ENABLED and self.r1_coast_prepare.active
        and session.allowed and brake.state == BrakeState.READY):
      command.speed = float(CS.out.vEgo)
      command.is_accel = False
      command.is_decel = False
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
      self.coast_release_counter = 0
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
