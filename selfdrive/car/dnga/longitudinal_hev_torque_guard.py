"""DNGA V4.3.7 HEV torque-neutralized longitudinal handoff.

Builds on the V4.3.6 openpilot-only policy. The HEV controller keeps 0x273
coherent with deceleration, actively drives hybrid torque toward a near-neutral
window while 0x271 friction braking is active, and latches a non-rising speed
anchor across brake release so stored positive drive torque cannot launch the
car as hydraulic braking disappears.
"""

from common.numpy_fast import clip, interp
from selfdrive.car.dnga.longitudinal import powertrain_decel_cap
from selfdrive.car.dnga.longitudinal_op_only import LongitudinalController as OpenpilotOnlyLongitudinalController
from selfdrive.car.dnga.values import BrakeState, LongitudinalParams as P


# Feedback values are raw, empirically correlated HEV signals. These limits do
# not claim engineering units; they only describe the near-neutral transition
# window observed in the Yaris Cross HEV logs.
HEV_POSITIVE_TORQUE_START = 120.0
HEV_NEGATIVE_TORQUE_START = -120.0
HEV_RPM_REGEN_GUARD = 1800.0
HEV_HYDRAULIC_NEUTRAL_BIAS = 0.04  # m/s below vEgo while friction carries decel
HEV_OFFSET_STEP_DOWN = 0.055        # m/s per 20 Hz longitudinal update
HEV_OFFSET_STEP_UP = 0.070
HEV_RPM_OFFSET_STEP_UP = 0.100
HEV_NEUTRAL_OFFSET_STEP = 0.025
HEV_RELEASE_MAX_RAW = 300.0
HEV_RELEASE_MAX_11BIT = 25.0

HEV_OFFSET_MAX_BP = [0.0, 5.0, 15.0, 25.0, 35.0]
HEV_OFFSET_MAX_V = [0.06, 0.10, 0.22, 0.35, 0.45]
HEV_POSITIVE_TORQUE_BP = [120.0, 300.0, 600.0, 1000.0, 1500.0]
HEV_POSITIVE_OFFSET_V = [0.05, 0.09, 0.18, 0.30, 0.45]


class LongitudinalController(OpenpilotOnlyLongitudinalController):
  def __init__(self):
    super().__init__()
    self.hev_last_v_ego = 0.0
    self.hev_release_anchor = 0.0
    self.hev_release_anchor_active = False

  def _clear_hev_release_anchor(self):
    self.hev_release_anchor = 0.0
    self.hev_release_anchor_active = False

  def _start_staged_release(self, frame):
    # Freeze the release target at the speed where braking finished. If the car
    # accelerates from stored positive torque, the 0x273 target must not follow
    # that acceleration upward.
    if self.hev_release_anchor_active:
      self.hev_release_anchor = min(self.hev_release_anchor, self.hev_last_v_ego)
    else:
      self.hev_release_anchor = max(0.0, self.hev_last_v_ego)
      self.hev_release_anchor_active = True
    super()._start_staged_release(frame)

  def _update_near_set_coast(self, CS, plan, lead_state, apply_accel):
    # V4.3.5 logs showed that a tiny no-lead negative target can enter a much
    # stronger HEV regen/engine-braking state. No-lead cruise now coasts through
    # the normal planner/PID path instead of synthesizing this extra decel bias.
    self.near_set_coast_active = False
    self.near_set_coast_counter = 0
    return False

  @staticmethod
  def _hev_offset_cap(v_ego):
    return float(interp(v_ego, HEV_OFFSET_MAX_BP, HEV_OFFSET_MAX_V))

  @staticmethod
  def _hev_torque_metrics(feedback):
    # 73/6 is the observed 0x2C9:0x12A relation already used by the feedback
    # consistency check. Put all four observations on a comparable raw scale.
    torque_12a_scaled = float(feedback["torque_12a"]) * 73.0 / 6.0
    torque_125_scaled = float(feedback["torque_125"]) * 73.0 / 6.0
    values = (
      float(feedback["torque_request"]),
      float(feedback["torque_actual"]),
      torque_12a_scaled,
      torque_125_scaled,
    )
    return max(values), min(values)

  def _hev_neutralized_offset(self, CS, session, propulsion, plan, brake):
    """Return a non-positive target offset that drives HEV torque toward neutral."""
    v_ego = float(CS.out.vEgo)
    max_offset = self._hev_offset_cap(v_ego)
    t_lookup = 0.35 + 0.07 * v_ego

    if brake.hydraulic:
      # Friction braking owns the requested deceleration. Keep only a tiny
      # negative powertrain bias, then let measured HEV torque move it as needed.
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
      # A completed decel can remain handoff_pending while torque settles. Do
      # not synthesize more decel demand during that feedback-only phase.
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
        # Positive drive torque while OP is decelerating must be actively
        # removed before friction release. Increase the below-vEgo target in a
        # bounded way, proportional to the observed unwanted torque.
        positive_offset = float(interp(
          positive_torque,
          HEV_POSITIVE_TORQUE_BP,
          HEV_POSITIVE_OFFSET_V,
        ))
        target_offset = -min(max_offset, max(abs(nominal_offset), positive_offset))
        step_down = HEV_OFFSET_STEP_DOWN
      elif (
        negative_torque < HEV_NEGATIVE_TORQUE_START
        or brake_request < -100.0
        or rpm_regen
      ):
        # Strong negative torque / 0x275 braking with rising engine RPM means
        # the powertrain is doing more decel than desired. Relax the 0x273
        # target toward neutral and let 0x271 friction carry the remaining load.
        target_offset = 0.0
        step_up = HEV_RPM_OFFSET_STEP_UP if rpm_regen else HEV_OFFSET_STEP_UP

    current_offset = float(clip(self.speed_offset, -max_offset, 0.0))
    if target_offset < current_offset:
      current_offset = max(target_offset, current_offset - step_down)
    else:
      current_offset = min(target_offset, current_offset + step_up)
    return float(clip(current_offset, -max_offset, 0.0))

  @staticmethod
  def _hev_strict_torque_ready(feedback):
    return (
      -100.0 <= float(feedback["torque_request"]) <= HEV_RELEASE_MAX_RAW
      and -100.0 <= float(feedback["torque_actual"]) <= HEV_RELEASE_MAX_RAW
      and -8.0 <= float(feedback["torque_12a"]) <= HEV_RELEASE_MAX_11BIT
      and -8.0 <= float(feedback["torque_125"]) <= HEV_RELEASE_MAX_11BIT
    )

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

    # The inherited V4.3.5 gate allowed up to +1000 raw torque. The V4.3.5
    # drive proved that several hundred counts of preloaded positive torque can
    # still produce a launch when friction disappears. Keep an existing handoff
    # latched until the four correlated torque observations are much nearer zero.
    if (
      session.enabled
      and was_handoff
      and (
        not session.feedback_clean
        or not self._hev_strict_torque_ready(session.feedback)
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

    # Active braking or a still-negative target is never allowed to consume the
    # ready debounce. Once the target is neutral and feedback is clean, the
    # inherited handoff observer can count its normal HYBRID_READY_COUNT samples.
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
    elif (brake.release_pump or self.handoff_pending) and self.hev_release_anchor_active:
      release_base = min(float(CS.out.vEgo), self.hev_release_anchor)
      command.speed = max(0.0, release_base + min(0.0, self.speed_offset))
    else:
      command.speed = max(0.0, float(CS.out.vEgo) + min(0.0, self.speed_offset))

    # One coherent DNGA mode: any negative target, hydraulic brake, release, or
    # torque-gated handoff is DECEL. ACCEL is not re-armed until the HEV has
    # actually returned to the tightened near-neutral feedback window.
    command.is_accel = False
    command.is_decel = True
    return command

  def update(self, enabled, CS, frame, accel, pcm_cancel_cmd, lead):
    self.hev_last_v_ego = max(0.0, float(CS.out.vEgo))
    if not enabled:
      self._clear_hev_release_anchor()
    return super().update(enabled, CS, frame, accel, pcm_cancel_cmd, lead)
