"""DNGA V7.0 stock-longitudinal controller.

Toyota's original camera 0x271/0x273/0x274 messages are forwarded by the
matching dedicated Panda safety hook. OP transmits only 0x1D0 steering.
The factory brake smoothing candidate is *shadow only*: it is never sent.
"""

from common.numpy_fast import clip, interp
from opendbc.can.packer import CANPacker
from selfdrive.car.dnga.dngacan import create_can_steer_command
from selfdrive.car.dnga.stock_long_shadow import StockBrakeShadow
from selfdrive.car.dnga.values import DBC, CarControllerParams

try:
  from selfdrive.swaglog import cloudlog
except ImportError:
  cloudlog = None


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
    self.stock_ldw = False
    self.stock_brake_shadow = StockBrakeShadow()

    # Never silently use this controller with OP longitudinal enabled:
    # its sole-owner camera forwarding needs a different Panda safety model.
    if CP.openpilotLongitudinalControl or str(CP.safetyConfigs[0].safetyModel) not in ("dngaStock", "32"):
      raise RuntimeError("V7 stocklong requires factory longitudinal and dngaStock Panda safety")

  def update(
    self, enabled, active, CS, frame, actuators, pcm_cancel_cmd,
    hud_alert, left_line, right_line, lead, left_lane_depart,
    right_lane_depart, dragonconf,
  ):
    can_sends = []

    steer_max_interp = max(1.0, interp(CS.out.vEgo, self.params.STEER_BP, self.params.STEER_LIM_TORQ))
    new_steer = int(round(actuators.steer * steer_max_interp))
    blinker_on = CS.out.leftBlinker != CS.out.rightBlinker
    apply_steer = apply_dnga_steer_torque_limits(
      new_steer, self.last_steer, CS.out.steeringTorqueEps, blinker_on, self.params
    )
    self.steer_rate_limited = new_steer != apply_steer and apply_steer != 0
    self.steer_rate_limited &= not CS.out.steeringPressed

    if frame % CarControllerParams.STEER_STEP == 0:
      steer_req = (enabled or self.stock_ldw) and CS.lkas_latch
      can_sends.append(
        create_can_steer_command(
          self.packer, apply_steer, steer_req,
          frame // CarControllerParams.STEER_STEP % 16
        )
      )

    # Observe the OEM camera request; never feed the proposal to the ECU.
    if frame % CarControllerParams.ACC_STEP == 0:
      observation = self.stock_brake_shadow.observe(
        frame,
        getattr(CS, "stock_acc_brake_rx_frame", -1000000),
        getattr(CS, "stock_acc_brake_state", -1),
        getattr(CS, "stock_brake_mag", 200),
      )
      if cloudlog is not None and frame % 100 == 0 and observation.valid:
        cloudlog.info(
          "V7_STOCKLONG_SHADOW raw_state=0x%02X raw_mag=%d hypothetical_mag=%d "
          "reason=%s (NOT TRANSMITTED)",
          observation.stock_state, observation.raw_magnitude,
          observation.proposed_magnitude, observation.reason,
        )

    # This is intentionally the complete transmit list. Stock ACC 0x271/0x273
    # and stock HUD 0x274 remain entirely OEM-owned by the dedicated Panda
    # forwarding hook. No OP curve brake, no synthetic lead, no ACC speed
    # target, no pump override, no active brake magnitude scaling.
    self.last_steer = apply_steer
    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / steer_max_interp
    new_actuators.accel = 0.0
    return new_actuators, can_sends
