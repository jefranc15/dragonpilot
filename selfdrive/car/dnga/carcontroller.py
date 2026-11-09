from common.numpy_fast import clip, interp
from opendbc.can.packer import CANPacker
from selfdrive.car import make_can_msg
from selfdrive.car.dnga.dngacan import (
  create_can_steer_command,
  dnga_create_accel_command,
  dnga_create_brake_command,
  dnga_create_hud,
)
from selfdrive.car.dnga.dnga_hybrid_feedback import hybrid_brake_feedback_state, hybrid_feedback_snapshot
from selfdrive.car.dnga.longitudinal_kinematic_guard import LongitudinalController
from selfdrive.car.dnga.r2_v4_shadow_controller import R2V4ShadowController
from selfdrive.car.dnga.r2_v4_direct_authority import R2V4DirectAuthority
from selfdrive.swaglog import cloudlog
from selfdrive.car.dnga.values import BrakeState, DBC, CarControllerParams

# BENCH/OFFLINE DEVELOPMENT ONLY. No vehicle-level response validation yet.
# This branch must not activate unvalidated direct R2 CAN actuation by default.
R2_V4_DIRECT_CAN_ENABLED = False

try:
  from common.features import Features
except ImportError:

  class Features:
    def has(self, feature_name):
      return False


def apply_dnga_steer_torque_limits(apply_torque, apply_torque_last, driver_torque, blinker_on, LIMITS):
  reduced_torque_mult = 10 if blinker_on else 1.5
  driver_max_torque = 255 + driver_torque * reduced_torque_mult
  driver_min_torque = -255 - driver_torque * reduced_torque_mult
  max_steer_allowed = clip(driver_max_torque, 0, 255)
  min_steer_allowed = clip(driver_min_torque, -255, 0)
  apply_torque = clip(apply_torque, min_steer_allowed, max_steer_allowed)
  if apply_torque_last > 0:
    apply_torque = clip(
      apply_torque,
      max(apply_torque_last - LIMITS.STEER_DELTA_DOWN, -LIMITS.STEER_DELTA_UP),
      apply_torque_last + LIMITS.STEER_DELTA_UP,
    )
  else:
    apply_torque = clip(
      apply_torque,
      apply_torque_last - LIMITS.STEER_DELTA_UP,
      min(apply_torque_last + LIMITS.STEER_DELTA_DOWN, LIMITS.STEER_DELTA_UP),
    )
  return int(round(float(apply_torque)))


class CarController:
  def __init__(self, dbc_name, CP, VM):
    self.last_steer = 0
    self.steer_rate_limited = False
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(DBC[CP.carFingerprint]["pt"])
    f = Features()
    self.need_clear_engine = f.has("ClearCode")
    self.stock_ldw = False
    self.longitudinal = LongitudinalController()
    self.r2_v4_shadow = R2V4ShadowController()  # Proposal generator; no direct CAN
    self.r2_v4_direct = R2V4DirectAuthority()  # Independent moving-brake producer
    self.r2_v4_last = None
    self.r2_v4_shadow_errors = 0

  def _apply_brake_propulsion_interlock(self, command, CS, frame):
    """Final brake/propulsion invariant for the V5.0 coast-first policy.

    0x275 brake request plus 0x08C friction remain a hard physical-braking
    acceleration veto. Raw torque request/actual channels have no authority.
    """
    feedback = hybrid_feedback_snapshot(CS, frame)
    _, physical_braking = hybrid_brake_feedback_state(feedback)

    if command.brake_state == BrakeState.HOLD:
      return command

    hydraulic = command.brake_state in (BrakeState.BRAKING, BrakeState.CREEPING)
    release_pump = command.pump < 0.0

    if physical_braking:
      # Defense in depth: physical regen/friction can never share an ACCEL
      # command, even if an upstream handoff transition clears too early.
      self.longitudinal.handoff_pending = True
      self.longitudinal.handoff_ready_counter = 0
      self.longitudinal.kinematic_ready_counter = 0

    if hydraulic or release_pump:
      # Commanded 0x271 braking/release keeps DECEL framing, with a neutral
      # 0x273 target so the powertrain is not asked for extra slowing.
      command.is_accel = False
      command.is_decel = True
      command.speed = float(CS.out.vEgo)
      self.longitudinal.speed_offset = 0.0
    elif physical_braking or self.longitudinal.handoff_pending:
      # After 0x271 releases, keep the hard accel veto but use true neutral
      # framing. This allows lingering regen/engine braking to unwind instead
      # of holding IS_DECEL high.
      command.is_accel = False
      command.is_decel = False
      command.speed = float(CS.out.vEgo)
      self.longitudinal.speed_offset = 0.0

    return command

  def _apply_r2_final_brake_interlock(self, command, CS, frame):
    """Final veto AFTER direct R2 selection; preserve R2's lower ACC target.

    Never interpret 0x275 as independently validated wheel torque; this
    remains a command/feedback guard, not proof of physical brake response.
    """
    feedback = hybrid_feedback_snapshot(CS, frame)
    fresh, braking_feedback = hybrid_brake_feedback_state(feedback)
    if command.brake_state == BrakeState.HOLD:
      return command
    if command.brake_state in (BrakeState.BRAKING, BrakeState.CREEPING) or command.pump < 0.0:
      command.is_accel = False
      command.is_decel = True
    elif braking_feedback or not fresh:
      command.is_accel = False
      command.is_decel = False
      command.speed = float(CS.out.vEgo)
    return command

  def _observe_r2_v4(self, enabled, CS, frame, actuators, pcm_cancel_cmd, command):
    """Diagnostic-only observer; never modifies output, controller, or CAN."""
    try:
      ctrl = self.longitudinal
      plan_frame = int(getattr(ctrl, "plan_frame", -1000000))
      source = str(getattr(ctrl, "plan_source", "")).lower().split(".")[-1]
      plan_ok = 0 <= frame - plan_frame <= 25
      current = 0.7 * float(getattr(ctrl, "plan_accel", 0.0)) + 0.3 * float(getattr(ctrl, "plan_accel_next", 0.0))
      preview = current
      if source == "turn":
        preview = min(current, float(getattr(ctrl, "plan_curve_accel", current)))
      elif source in ("lead0", "lead1"):
        preview = min(current, float(getattr(ctrl, "plan_lead_accel", current)))
      rx275 = int(getattr(CS, "hybrid_feedback_rx_frame_275", -1000000))
      rx08c = int(getattr(CS, "hybrid_feedback_rx_frame_08c", -1000000))
      fresh = (0 <= frame-rx275 <= 25 and 0 <= frame-rx08c <= 25)
      session = bool(enabled and CS.out.cruiseState.enabled and plan_ok and not pcm_cancel_cmd)
      result = self.r2_v4_shadow.step(
        frame=frame, v_ego_mps=float(CS.out.vEgo),
        planned_accel=current, preview_accel=preview,
        downstream_pid=float(actuators.accel),
        measured_accel=float(CS.out.aEgo), source=source,
        has_lead=bool(getattr(ctrl, "plan_has_lead", False)), enabled=session,
        driver_override=bool(CS.out.brakePressed or CS.out.gasPressed),
        feedback_275=int(getattr(CS, "hybrid_brake_request_raw_275", 0)),
        friction_08c=int(getattr(CS, "hybrid_friction_raw_08c", 0)),
        feedback_fresh=fresh, feedback_sequence=(rx275, rx08c),
        positive_accel_permitted=bool(actuators.accel > 0.05),
        urgent=bool(getattr(ctrl, "urgent_brake", False)),
      )
      self.r2_v4_last = result
      if session and frame % 100 == 0:
        cloudlog.info(
          "R2V4_SHADOW_ONLY f=%d src=%s actual_state=%d actual_mag=%d actual_target_mps=%.3f "
          "would_state=%d would_mag=%d would_target_kph=%.2f gap=%.2f bridge=%d out_of_stock=%d miss275=%d",
          frame, source, command.brake_state, command.magnitude, command.speed,
          result.brake_state, result.magnitude, result.target_kph, result.target_gap_kph,
          result.source_bridge, result.outside_stock_calibration, result.missing_275_response,
        )
      return result
    except Exception as exc:
      self.r2_v4_shadow_errors += 1
      if frame % 100 == 0:
        cloudlog.warning("R2V4_SHADOW_ONLY observer error: %r", exc)
      return None

  def update(
    self,
    enabled,
    active,
    CS,
    frame,
    actuators,
    pcm_cancel_cmd,
    hud_alert,
    left_line,
    right_line,
    lead,
    left_lane_depart,
    right_lane_depart,
    dragonconf,
  ):
    can_sends = []
    steer_max_interp = interp(CS.out.vEgo, self.params.STEER_BP, self.params.STEER_LIM_TORQ)
    steer_max_interp = max(1.0, steer_max_interp)
    new_steer = int(round(actuators.steer * steer_max_interp))
    blinker_on = CS.out.leftBlinker != CS.out.rightBlinker
    apply_steer = apply_dnga_steer_torque_limits(
      new_steer, self.last_steer, CS.out.steeringTorqueEps, blinker_on, self.params
    )
    self.steer_rate_limited = new_steer != apply_steer and apply_steer != 0
    self.steer_rate_limited &= not CS.out.steeringPressed
    if self.need_clear_engine:
      can_sends.append(make_can_msg(2015, b"\x01\x04\x00\x00\x00\x00\x00\x00", 0))
    if frame % CarControllerParams.STEER_STEP == 0:
      steer_req = (enabled or self.stock_ldw) and CS.lkas_latch
      can_sends.append(
        create_can_steer_command(self.packer, apply_steer, steer_req, frame // CarControllerParams.STEER_STEP % 16)
      )
    command = self.longitudinal.update(enabled, CS, frame, actuators.accel, pcm_cancel_cmd, lead)
    if command is not None:
      # Compute the original V5.0 command and its existing brake-feedback veto.
      command = self._apply_brake_propulsion_interlock(command, CS, frame)
      v50_command = command
      # R2 V4 actual moving-braking experiment: coordinate 0x271 and 0x273.
      proposal = self._observe_r2_v4(enabled, CS, frame, actuators, pcm_cancel_cmd, v50_command)
      # Select EXACTLY ONE longitudinal command authority. The previous
      # V5.2 command combiner is deliberately removed: V5.0's coast-first
      # command is never used as the normal R2 hydraulic demand/floor.
      if R2_V4_DIRECT_CAN_ENABLED:
        feedback = hybrid_feedback_snapshot(CS, frame)
        fresh, braking_feedback = hybrid_brake_feedback_state(feedback)
        command, r2_status = self.r2_v4_direct.apply(
          v50_command, proposal,
          enabled=bool(enabled and CS.out.cruiseState.enabled and not pcm_cancel_cmd),
          speed_mps=float(CS.out.vEgo),
          override=bool(CS.out.brakePressed or CS.out.gasPressed),
          urgent=bool(getattr(self.longitudinal, "urgent_brake", False)),
          engagement_guard=(frame < int(getattr(self.longitudinal, "block_brake_until_frame", 0))),
          feedback_fresh=fresh,
          friction_active=braking_feedback,
        )
        command = self._apply_r2_final_brake_interlock(command, CS, frame)
        if frame % 20 == 0 or r2_status.feedback_fault:
          cloudlog.info(
            "R2V4_DIRECT f=%d authority=%s reason=%s mag=%d speed=%.3f "
            "accel=%d decel=%d expected275=%d 275raw=%d friction=%d fault=%s",
            frame, r2_status.authority, r2_status.reason,
            command.magnitude, command.speed, command.is_accel,
            command.is_decel, r2_status.expected_275,
            int(getattr(CS, "hybrid_brake_request_raw_275", 0)),
            int(getattr(CS, "hybrid_friction_raw_08c", 0)),
            r2_status.feedback_fault,
          )
      can_sends.append(
        dnga_create_accel_command(
          self.packer,
          CS.cruise_speed,
          CS.out.cruiseState.available,
          command.enabled,
          command.lead,
          command.speed,
          command.is_accel,
          command.is_decel,
          CS.op_distance_val,
        )
      )
      can_sends.append(
        dnga_create_brake_command(
          self.packer, command.brake_state, command.pump, command.magnitude, frame // CarControllerParams.ACC_STEP % 8
        )
      )
      can_sends.append(
        dnga_create_hud(
          self.packer,
          CS.out.cruiseState.available and CS.lkas_latch,
          enabled,
          left_line,
          right_line,
          self.stock_ldw,
          CS.stock_fcw,
          CS.stock_aeb,
          CS.stock_adas_frontDepartureHUD,
          CS.stock_lkc_off,
          CS.stock_fcw_off,
        )
      )
    self.last_steer = apply_steer
    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / steer_max_interp
    return (new_actuators, can_sends)
