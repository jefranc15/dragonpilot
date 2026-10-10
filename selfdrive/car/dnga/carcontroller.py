from common.numpy_fast import clip, interp
from opendbc.can.packer import CANPacker
from selfdrive.car import make_can_msg
from selfdrive.car.dnga.dngacan import (
  create_can_steer_command,
  dnga_create_accel_command,
  dnga_create_brake_command,
  dnga_create_hud,
)
from selfdrive.car.dnga.dnga_hybrid_feedback import hybrid_feedback_snapshot
from selfdrive.car.dnga.longitudinal_hev_torque_guard import LongitudinalController
from selfdrive.car.dnga.values import BrakeState, DBC, CarControllerParams, LongitudinalParams as P

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
    # Final command-layer brake/propulsion interlock. Normal stock ACC requests
    # remain logging-only; the HEV-aware controller owns the torque-neutralized
    # decel/release target and this layer remains defense in depth.
    self.brake_propulsion_lock = False
    self.brake_propulsion_ready_count = 0

  def _apply_brake_propulsion_interlock(self, command, CS, frame):
    """Prevent positive 0x273 propulsion while the HEV is physically braking.

    Stock ACC 0x271/0x273 remains decoded for logging/comparison but has no
    normal longitudinal authority. Physical hybrid feedback remains the final
    external brake-to-propulsion gate.
    """
    feedback = hybrid_feedback_snapshot(CS, frame)
    feedback_clean = feedback["fresh"] and feedback["consistent"]
    physical_braking = feedback_clean and (not feedback["brakes_clear"])

    if physical_braking:
      self.brake_propulsion_lock = True
      self.brake_propulsion_ready_count = 0
    elif self.brake_propulsion_lock:
      ready_candidate = (
        feedback_clean
        and feedback["brakes_clear"]
        and feedback["torque_ramp_ready"]
      )
      if ready_candidate:
        self.brake_propulsion_ready_count = min(
          self.brake_propulsion_ready_count + 1,
          P.HYBRID_READY_COUNT,
        )
      else:
        self.brake_propulsion_ready_count = 0
      if self.brake_propulsion_ready_count >= P.HYBRID_READY_COUNT:
        self.brake_propulsion_lock = False
        self.brake_propulsion_ready_count = 0

    # Hydraulic/crawl braking can never advertise ACCEL. HOLD keeps the
    # stock-observed both-bits mode. This is a final invariant even if an
    # upstream state machine regresses later.
    if command.brake_state in (BrakeState.BRAKING, BrakeState.CREEPING):
      command.is_accel = False
      command.is_decel = True
      command.speed = min(float(command.speed), float(CS.out.vEgo))

    if self.brake_propulsion_lock and command.brake_state != BrakeState.HOLD:
      command.is_accel = False
      command.is_decel = True
      command.speed = min(float(command.speed), float(CS.out.vEgo))

      # Do not let a positive desired-speed offset survive underneath the
      # interlock and launch the car when the physical brake finally clears.
      self.longitudinal.speed_offset = min(0.0, float(self.longitudinal.speed_offset))
      self.longitudinal.handoff_pending = True
      self.longitudinal.handoff_ready_counter = 0

    return command

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
      command = self._apply_brake_propulsion_interlock(command, CS, frame)
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
