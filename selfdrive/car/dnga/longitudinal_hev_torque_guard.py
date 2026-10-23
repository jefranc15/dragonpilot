"""DNGA V4.3.8 HEV torque-neutralized longitudinal handoff.

Builds on V4.3.7 with a torque-gated soft hydraulic release, a larger but
feedback-bounded 0x273 decel target when positive HEV torque refuses to unwind,
and a release anchor that is strictly scoped to one release transition.
"""

from common.numpy_fast import clip, interp
from selfdrive.car.dnga.longitudinal import powertrain_decel_cap
from selfdrive.car.dnga.longitudinal_op_only import LongitudinalController as OpenpilotOnlyLongitudinalController
from selfdrive.car.dnga.values import BrakeState, LongitudinalParams as P


# Raw empirically correlated HEV signals. These are transition thresholds, not
# claimed engineering units.
HEV_POSITIVE_TORQUE_START = 120.0
HEV_NEGATIVE_TORQUE_START = -120.0
HEV_RPM_REGEN_GUARD = 1800.0
HEV_HYDRAULIC_NEUTRAL_BIAS = 0.04
HEV_OFFSET_STEP_DOWN = 0.100
HEV_OFFSET_STEP_UP = 0.100
HEV_RPM_OFFSET_STEP_UP = 0.150
HEV_NEUTRAL_OFFSET_STEP = 0.035

# Propulsion handoff is stricter than V4.3.7. The hydraulic release guard only
# cares about positive preloaded torque; strong negative torque may release
# friction but still cannot re-arm propulsion until this full near-neutral
# window is satisfied.
HEV_HANDOFF_MAX_RAW = 225.0
HEV_HANDOFF_MAX_11BIT = 18.0
HEV_RELEASE_POSITIVE_MAX_RAW = 225.0
HEV_RELEASE_POSITIVE_MAX_11BIT = 18.0
HEV_RELEASE_READY_COUNT = 4
HEV_RELEASE_GUARD_MIN_SPEED = 5.0

# V4.3.7 saturated near ~1 km/h below vEgo while +1000 raw torque persisted.
# Stock ACC and the accidental stale-anchor capture both show that this HEV may
# need roughly 4-6 km/h of temporary target separation to unload positive drive
# torque. V4.3.8 admits that range only while friction braking is already active
# and measured positive torque demands it; negative torque/RPM immediately
# drives the target back toward neutral.
HEV_OFFSET_MAX_BP = [0.0, 5.0, 15.0, 25.0, 35.0]
HEV_OFFSET_MAX_V = [0.08, 0.30, 1.20, 1.75, 1.90]
HEV_POSITIVE_TORQUE_BP = [120.0, 250.0, 400.0, 600.0, 800.0, 1000.0, 1250.0, 1500.0]
HEV_POSITIVE_OFFSET_V = [0.05, 0.12, 0.30, 0.62, 0.98, 1.45, 1.75, 1.90]


class LongitudinalController(OpenpilotOnlyLongitudinalController):
  def __init__(self):
    super().__init__()
    self.hev_last_v_ego = 0.0
    self.hev_release_anchor = 0.0
    self.hev_release_anchor_active = False
    self.hev_release_ready_counter = 0

  def _clear_hev_release_anchor(self):
    self.hev_release_anchor = 0.0
    self.hev_release_anchor_active = False

  def _start_staged_release(self, frame):
    # A release anchor belongs to exactly one release. Always reseed it from the
    # current speed instead of inheriting/min()ing an older transition.
    self.hev_release_anchor = max(0.0, self.hev_last_v_ego)
    self.hev_release_anchor_active = True
    super()._start_staged_release(frame)

  def _update_near_set_coast(self, CS, plan, lead_state, apply_accel):
    # Tiny synthetic no-lead negative targets can trigger disproportionate HEV
    # regen/engine braking. Leave no-lead cruise to the normal planner/PID path.
    self.near_set_coast_active = False
    self.near_set_coast_counter = 0
    return False

  @staticmethod
  def _hev_offset_cap(v_ego):
    return float(interp(v_ego, HEV_OFFSET_MAX_BP, HEV_OFFSET_MAX_V))

  @staticmethod
  def _hev_torque_metrics(feedback):
    # 73/6 is the observed 0x2C9:0x12A relation used by the consistency check.
    torque_12a_scaled = float(feedback["torque_12a"]) * 73.0 / 6.0
    torque_125_scaled = float(feedback["torque_125"]) * 73.0 / 6.0
    values = (
      float(feedback["torque_request"]),
      float(feedback["torque_actual"]),
      torque_12a_scaled,
      torque_125_scaled,
    )
    return max(values), min(values)

  @staticmethod
  def hev_positive_release_ready(feedback):
    """True when no correlated torque channel still shows launch-sized drive torque."""
    return (
      float(feedback["torque_request"]) <= HEV_RELEASE_POSITIVE_MAX_RAW
      and float(feedback["torque_actual"]) <= HEV_RELEASE_POSITIVE_MAX_RAW
      and float(feedback["torque_12a"]) <= HEV_RELEASE_POSITIVE_MAX_11BIT
      and float(feedback["torque_125"]) <= HEV_RELEASE_POSITIVE_MAX_11BIT
    )

  @staticmethod
  def hev_handoff_torque_ready(feedback):
    """Full near-neutral window required before positive propulsion can re-arm."""
    return (
      -100.0 <= float(feedback["torque_request"]) <= HEV_HANDOFF_MAX_RAW
      and -100.0 <= float(feedback["torque_actual"]) <= HEV_HANDOFF_MAX_RAW
      and -8.0 <= float(feedback["torque_12a"]) <= HEV_HANDOFF_MAX_11BIT
      and -8.0 <= float(feedback["torque_125"]) <= HEV_HANDOFF_MAX_11BIT
    )

  def _hev_neutralized_offset(self, CS, session, propulsion, plan, brake):
    """Return a non-positive target offset that actively unloads HEV drive torque."""
    v_ego = float(CS.out.vEgo)
    max_offset = self._hev_offset_cap(v_ego)
    t_lookup = 0.35 + 0.07 * v_ego

    if brake.hydraulic:
      nominal_offset = -min(max_offset, HEV_HYDRAULIC_NEUTRAL_BIAS)
    elif brake.release_pump or brake.sng_release:
      nominal_offset = 0.0
    elif propulsion.accel <= -P.DECEL_DEADBAND:
      requested_decel = max(0.0, -float(propulsion.accel))
      nominal_offset = -min(
        max_offset,
        powertrain_decel_cap(v_ego),
        requested_decel * t_lookup,
      )
    else:
      nominal_offset = 0.0

    target_offset = nominal_offset
    step_down = HEV_NEUTRAL_OFFSET_STEP
    step_up = HEV_NEUTRAL_OFFSET_STEP

    if session.feedback_clean:
      positive_torque, negative_torque = self._hev_torque_metrics(session.feedback)
      rpm = float(getattr(CS.out, "engineRPM", 0.0))
      brake_request = float(session.feedback["brake_request"])
      rpm_regen = (
        rpm >= HEV_RPM_REGEN_GUARD
        and (negative_torque < -80.0 or brake_request < -100.0)
      )

      if positive_torque > HEV_POSITIVE_TORQUE_START:
        positive_offset = float(interp(
          positive_torque,
          HEV_POSITIVE_TORQUE_BP,
          HEV_POSITIVE_OFFSET_V,
        ))
        target_offset = -min(max_offset, max(abs(nominal_offset), positive_offset))
        step_down = HEV_OFFSET_STEP_DOWN
        step_up = HEV_OFFSET_STEP_UP
      elif (
        negative_torque < HEV_NEGATIVE_TORQUE_START
        or brake_request < -100.0
        or rpm_regen
      ):
        # Do not trade positive-torque fighting for excessive regen/engine
        # braking. As soon as feedback is genuinely negative, relax toward vEgo.
        target_offset = 0.0
        step_up = HEV_RPM_OFFSET_STEP_UP if rpm_regen else HEV_OFFSET_STEP_UP

    current_offset = float(clip(self.speed_offset, -max_offset, 0.0))
    if target_offset < current_offset:
      current_offset = max(target_offset, current_offset - step_down)
    else:
      current_offset = min(target_offset, current_offset + step_up)
    return float(clip(current_offset, -max_offset, 0.0))

  def _soft_release_guard_context(self, CS, session, plan, lead_state):
    """Only normal live-lead soft releases may be held for torque neutralization."""
    return (
      session.allowed
      and plan.fresh
      and plan.has_lead
      and lead_state.relevant
      and lead_state.status
      and float(CS.out.vEgo) >= HEV_RELEASE_GUARD_MIN_SPEED
    )

  def _update_hydraulic(self, CS, frame, session, apply_accel, plan, lead_state):
    was_brake_active = bool(self.brake_active)
    brake = super()._update_hydraulic(CS, frame, session, apply_accel, plan, lead_state)

    release_attempt = was_brake_active and brake.release_pump and not brake.sng_release
    if release_attempt and self._soft_release_guard_context(CS, session, plan, lead_state):
      if not session.feedback_clean:
        # Never hold hydraulic braking on stale/inconsistent passive feedback.
        # Fall back to the base hard/soft release behavior.
        self.hev_release_ready_counter = 0
        return brake

      release_ready = self.hev_positive_release_ready(session.feedback)
      if release_ready:
        self.hev_release_ready_counter = min(self.hev_release_ready_counter + 1, HEV_RELEASE_READY_COUNT)
      else:
        self.hev_release_ready_counter = 0

      if self.hev_release_ready_counter < HEV_RELEASE_READY_COUNT:
        # The base controller already staged/cleared the brake. Cancel that
        # release and keep only BRAKE_MIN while the stronger 0x273 target unloads
        # positive hybrid torque. Hard releases never enter this branch.
        self.release_pump_until_frame = frame
        self._clear_hev_release_anchor()
        self.brake_active = True
        self.curve_brake_active = False
        self.apply_brake = P.BRAKE_MIN
        self.brake_target = P.BRAKE_MIN
        self.release_counter = P.RELEASE_CONFIRM_COUNT
        self.brake_reentry_frame = frame
        self.handoff_pending = True
        self.handoff_ready_counter = 0
        brake.hydraulic = True
        brake.release_pump = False
        brake.sng_release = False
        return brake

      self.hev_release_ready_counter = 0
      return brake

    # Any real/new hydraulic phase invalidates an old release anchor. This is
    # defense in depth against the V4.3.7 stale-anchor failure.
    if brake.hydraulic:
      self.hev_release_ready_counter = 0
      self._clear_hev_release_anchor()
    elif not release_attempt:
      self.hev_release_ready_counter = 0

    return brake

  def _update_handoff_feedback(self, session, propulsion, brake):
    was_handoff = (
      self.handoff_pending
      or brake.hydraulic
      or brake.sng_release
      or brake.release_pump
      or propulsion.accel <= -P.DECEL_DEADBAND
      or self.speed_offset < -P.SPEED_OFFSET_EPS
    )
    super()._update_handoff_feedback(session, propulsion, brake)

    if (
      session.enabled
      and was_handoff
      and (
        not session.feedback_clean
        or not self.hev_handoff_torque_ready(session.feedback)
      )
    ):
      self.handoff_pending = True
      self.handoff_ready_counter = 0
      propulsion.ramp_ready = False

    if (
      not self.handoff_pending
      and not brake.hydraulic
      and not brake.sng_release
      and not brake.release_pump
    ):
      self._clear_hev_release_anchor()

  def _update_speed_offset(self, CS, frame, session, propulsion, plan, brake):
    if not session.allowed:
      self._clear_hev_release_anchor()
      return super()._update_speed_offset(CS, frame, session, propulsion, plan, brake)

    decel_phase = (
      brake.hydraulic
      or brake.sng_release
      or brake.release_pump
      or self.handoff_pending
      or propulsion.accel <= -P.DECEL_DEADBAND
    )
    if not decel_phase:
      return super()._update_speed_offset(CS, frame, session, propulsion, plan, brake)

    self.speed_offset = self._hev_neutralized_offset(CS, session, propulsion, plan, brake)

    if (
      brake.hydraulic
      or brake.sng_release
      or brake.release_pump
      or propulsion.accel <= -P.DECEL_DEADBAND
      or self.speed_offset < -P.SPEED_OFFSET_EPS
    ):
      self.handoff_pending = True
      self.handoff_ready_counter = 0

  def _build_command(self, CS, frame, session, plan, lead_state, brake, propulsion):
    command = super()._build_command(CS, frame, session, plan, lead_state, brake, propulsion)

    if not session.enabled or session.gas_override or brake.state == BrakeState.HOLD:
      return command

    decel_mode = (
      brake.hydraulic
      or brake.sng_release
      or brake.release_pump
      or self.handoff_pending
      or self.speed_offset < -P.SPEED_OFFSET_EPS
      or propulsion.accel <= -P.DECEL_DEADBAND
    )
    if not decel_mode:
      return command

    if brake.sng_release:
      command.speed = 0.0
    elif brake.hydraulic:
      # Never apply an old release anchor to a current hydraulic event.
      command.speed = max(0.0, float(CS.out.vEgo) + min(0.0, self.speed_offset))
    elif self.hev_release_anchor_active and (brake.release_pump or self.handoff_pending):
      release_base = min(float(CS.out.vEgo), self.hev_release_anchor)
      command.speed = max(0.0, release_base + min(0.0, self.speed_offset))
    else:
      command.speed = max(0.0, float(CS.out.vEgo) + min(0.0, self.speed_offset))

    command.is_accel = False
    command.is_decel = True
    return command

  def update(self, enabled, CS, frame, accel, pcm_cancel_cmd, lead):
    self.hev_last_v_ego = max(0.0, float(CS.out.vEgo))
    if not enabled:
      self.hev_release_ready_counter = 0
      self._clear_hev_release_anchor()
    return super().update(enabled, CS, frame, accel, pcm_cancel_cmd, lead)
