from selfdrive.car import make_can_msg
from selfdrive.car.dnga.dngacan import create_can_steer_command, \
                                       dnga_create_accel_command, \
                                       dnga_create_brake_command, \
                                       dnga_create_hud
from selfdrive.car.dnga.values import DBC, NOT_CAN_CONTROLLED
from opendbc.can.packer import CANPacker
from common.numpy_fast import clip, interp
from selfdrive.config import Conversions as CV

try:
  from common.features import Features
except ImportError:
  class Features:
    def has(self, feature_name):
      return False


def apply_dnga_steer_torque_limits(apply_torque, apply_torque_last, driver_torque, blinkerOn, LIMITS):
  reduced_torque_mult = 10 if blinkerOn else 1.5

  driver_max_torque = 255 + driver_torque * reduced_torque_mult
  driver_min_torque = -255 - driver_torque * reduced_torque_mult

  max_steer_allowed = clip(driver_max_torque, 0, 255)
  min_steer_allowed = clip(driver_min_torque, -255, 0)

  apply_torque = clip(apply_torque, min_steer_allowed, max_steer_allowed)

  if apply_torque_last > 0:
    apply_torque = clip(
      apply_torque,
      max(apply_torque_last - LIMITS.STEER_DELTA_DOWN, -LIMITS.STEER_DELTA_UP),
      apply_torque_last + LIMITS.STEER_DELTA_UP
    )
  else:
    apply_torque = clip(
      apply_torque,
      apply_torque_last - LIMITS.STEER_DELTA_UP,
      min(apply_torque_last + LIMITS.STEER_DELTA_DOWN, LIMITS.STEER_DELTA_UP)
    )

  return int(round(float(apply_torque)))


class CarControllerParams():
  def __init__(self, CP):
    self.STEER_BP = CP.lateralParams.torqueBP
    self.STEER_LIM_TORQ = CP.lateralParams.torqueV

    if CP.carFingerprint in NOT_CAN_CONTROLLED:
      self.STEER_DELTA_UP = 20
      self.STEER_DELTA_DOWN = 30
    else:
      self.STEER_DELTA_UP = 17
      self.STEER_DELTA_DOWN = 30


class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.last_steer = 0
    self.steer_rate_limited = False

    self.params = CarControllerParams(CP)
    self.packer = CANPacker(DBC[CP.carFingerprint]['pt'])

    f = Features()
    self.need_clear_engine = f.has("ClearCode")

    self.stockLdw = False

    # Prevent instant braking right after SET / engage
    self.prev_enabled = False
    self.block_brake_until_frame = 0

  def update(self, enabled, active, CS, frame, actuators, pcm_cancel_cmd,
             hud_alert, left_line, right_line, lead,
             left_lane_depart, right_lane_depart, dragonconf):

    can_sends = []

    # -----------------------------
    # Steering
    # -----------------------------
    steer_max_interp = interp(CS.out.vEgo, self.params.STEER_BP, self.params.STEER_LIM_TORQ)
    steer_max_interp = max(1.0, steer_max_interp)

    new_steer = int(round(actuators.steer * steer_max_interp))

    isBlinkerOn = CS.out.leftBlinker != CS.out.rightBlinker

    apply_steer = apply_dnga_steer_torque_limits(
      new_steer,
      self.last_steer,
      CS.out.steeringTorqueEps,
      isBlinkerOn,
      self.params
    )

    self.steer_rate_limited = (new_steer != apply_steer) and (apply_steer != 0)
    self.steer_rate_limited &= not CS.out.steeringPressed

    # -----------------------------
    # Longitudinal base values
    # -----------------------------
    apply_accel = clip(actuators.accel, -3.0, 1.5)
    apply_brake = -apply_accel if apply_accel < 0.0 else 0.0

    # Block braking briefly after engagement
    if enabled and not self.prev_enabled:
      self.block_brake_until_frame = frame + 200  # about 2 sec at 100 Hz

    self.prev_enabled = enabled

    # Clear Engine Codes
    if self.need_clear_engine or frame < 1000:
      can_sends.append(make_can_msg(2015, b'\x01\x04\x00\x00\x00\x00\x00\x00', 0))

    # Steering command, 50 Hz
    if (frame % 2) == 0:
      steer_req = (enabled or self.stockLdw) and CS.lkas_latch
      can_sends.append(
        create_can_steer_command(
          self.packer,
          apply_steer,
          steer_req,
          (frame // 2) % 16
        )
      )

    # Longitudinal / HUD, 20 Hz
    if (frame % 5) == 0:

      # -----------------------------
      # Desired speed for ACC_CMD_HUD
      # -----------------------------
      boost = interp(CS.out.vEgo, [0.2, 0.5, 18.0, 23.0], [0.0, 1.0, 1.0, 1.0])
      base_speed = getattr(actuators, 'speed', CS.out.vEgo)
      des_speed = base_speed + min((actuators.accel * boost), 1.0)

      # -----------------------------
      # ACC_BRAKE / 0x271
      # -----------------------------
      # Stock neutral from your logs:
      # 00 00 00 00 00 C8 ...
      brake_state = 0x00
      pump_reaction = 0.0
      brake_mag = 200

      # Highway-only braking for now.
      # Do NOT use OP braking below 30 kph yet.
      highway_brake_allowed = (
        enabled and
        frame > self.block_brake_until_frame and
        CS.out.vEgo > 30.0 * CV.KPH_TO_MS and
        apply_brake > 0.25 and
        not CS.out.gasPressed and
        not CS.out.brakePressed
      )

      if highway_brake_allowed:
        # Stock active braking family from logs:
        # 00 21 00 FC 04 xx ...
        brake_state = 0x21
        pump_reaction = -0.4

        # Conservative first map.
        # Lower value seems stronger based on your stock logs.
        brake_mag = int(interp(
          clip(apply_brake, 0.25, 1.20),
          [0.25, 1.20],
          [1219, 1180]
        ))

      # Only tell ACC_CMD_HUD that we are decelerating when we actually send 0x21
      brake_amt_for_hud = apply_brake if highway_brake_allowed else 0.0

      # ACC_CMD_HUD / 0x273
      can_sends.append(
        dnga_create_accel_command(
          self.packer,
          CS.cruise_speed,
          CS.out.cruiseState.available,
          enabled,
          lead,
          des_speed,
          brake_amt_for_hud,
          CS.op_distance_val
        )
      )

      # ACC_BRAKE / 0x271
      can_sends.append(
        dnga_create_brake_command(
          self.packer,
          brake_state,
          pump_reaction,
          brake_mag,
          (frame // 5) % 8
        )
      )

      # LKAS_HUD / 0x274
      can_sends.append(
        dnga_create_hud(
          self.packer,
          CS.out.cruiseState.available and CS.lkas_latch,
          enabled,
          left_line,
          right_line,
          self.stockLdw,
          CS.stock_fcw,
          CS.stock_aeb,
          CS.stock_adas_frontDepartureHUD,
          CS.stock_lkc_off,
          CS.stock_fcw_off
        )
      )

    self.last_steer = apply_steer

    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / steer_max_interp

    return new_actuators, can_sends
