from selfdrive.car import make_can_msg
from selfdrive.car.dnga.dngacan import (
  create_can_steer_command,
  dnga_create_accel_command,
  dnga_create_brake_command,
  dnga_create_hud,
)
from selfdrive.car.dnga.values import DBC, NOT_CAN_CONTROLLED
from opendbc.can.packer import CANPacker
from common.numpy_fast import clip, interp

try:
  from common.features import Features
except ImportError:
  class Features:
    def has(self, feature_name):
      return False


# V2.5K HEV smooth longitudinal handoff.
#
# The existing HEV DBC exposes bytes 4-5 of ACC_BRAKE/0x271 as one 16-bit
# MAGNITUDE value. Stock traffic and BukaPilot's newer DNGA DBC show that the
# two bytes are actually separate fields:
#   byte 4: positive pump stage (0.4, 0.5, 0.6, 0.7)
#   byte 5: decel command encoded as physical_value = raw * 0.01 - 2.0
# PUMP_REACTION2 in byte 3 carries the matching negative pump stage.
#
# V2.5K deliberately stays inside the most common stock moving-brake region:
# pump -0.4 / +0.4 and 0.05-0.75 m/s^2. Stronger pump stages remain disabled
# until a dedicated stock-ACC-only capture verifies when the vehicle uses them.
#
# Hydraulic entry has a deadband, but an active brake episode is not dropped
# on the first zero/positive planner sample. Instead, command magnitude ramps
# to the known 0.05 minimum and releases only after a stable 0.75-second
# neutral/positive request. Positive propulsion remains inhibited during the
# episode and for 0.4 seconds after release.
V25K_BRAKE_MIN = 0.05
V25K_BRAKE_ENTRY = 0.20
V25K_BRAKE_URGENT_ENTRY = 0.50
V25K_BRAKE_MAX = 0.75
V25K_BRAKE_STEP_UP = 0.04
V25K_BRAKE_STEP_DOWN = 0.05
V25K_BRAKE_ENTRY_FRAMES = 2       # 0.10 seconds at 20 Hz
V25K_BRAKE_RELEASE_FRAMES = 15    # 0.75 seconds at 20 Hz
V25K_PROPULSION_DWELL_FRAMES = 40 # 0.40 seconds at the 100 Hz frame clock
V25K_MIN_MOVING_SPEED = 0.10      # m/s; stop/hold states are not used yet

V25K_ACCEL_ENTRY = 0.12
V25K_ACCEL_ENTRY_FRAMES = 3       # filter short positive planner spikes
V25K_SPEED_OFFSET_NEG_STEP = 0.05
V25K_SPEED_OFFSET_RECOVERY_STEP = 0.03
V25K_SPEED_OFFSET_POS_STEP = 0.01
V25K_SPEED_OFFSET_NEG_MAX = 1.20


def v25k_rate_limit_brake(target, last):
  if target > last:
    return min(target, last + V25K_BRAKE_STEP_UP)
  return max(target, last - V25K_BRAKE_STEP_DOWN)


def v25k_low_speed_brake_cap(v_ego):
  """Taper moving hydraulic authority smoothly toward zero vehicle speed."""
  return float(interp(
    v_ego,
    [0.10, 0.30, 0.60, 1.00, 1.50, 2.50],
    [0.05, 0.08, 0.15, 0.28, 0.45, 0.75],
  ))


def v25k_update_speed_offset(target, last):
  """Asymmetric rate limit for a smooth regen/propulsion speed command."""
  if target > last:
    # Recover a negative offset to neutral before allowing positive demand.
    if last < 0.0:
      return min(target, min(0.0, last + V25K_SPEED_OFFSET_RECOVERY_STEP))
    return min(target, last + V25K_SPEED_OFFSET_POS_STEP)
  return max(target, last - V25K_SPEED_OFFSET_NEG_STEP)


def v25k_encode_hev_brake(brake_cmd):
  """Return (negative pump reaction, legacy combined raw magnitude)."""
  brake_cmd = float(clip(brake_cmd, V25K_BRAKE_MIN, V25K_BRAKE_MAX))
  pump = 0.4

  magnitude_byte = int(round(200.0 - 100.0 * brake_cmd))
  magnitude_byte = int(clip(magnitude_byte, 0, 255))
  pump_byte = int(round(pump * 10.0))

  # Preserve the current DBC's 16-bit MAGNITUDE packing exactly.
  combined_magnitude = (pump_byte << 8) | magnitude_byte
  return -pump, combined_magnitude


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
      self.STEER_DELTA_UP = 22
      self.STEER_DELTA_DOWN = 35


class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.last_steer = 0
    self.steer_rate_limited = False

    self.params = CarControllerParams(CP)
    self.packer = CANPacker(DBC[CP.carFingerprint]['pt'])

    f = Features()
    self.need_clear_engine = f.has("ClearCode")

    self.stockLdw = False

    self.prev_enabled = False
    self.block_brake_until_frame = 0

    # V2.5K: one continuous, magnitude-ramped hydraulic episode survives brief
    # planner sign changes. Propulsion gets a separate, slower handoff.
    self.v25k_apply_brake = 0.0
    self.v25k_brake_active = False
    self.v25k_brake_entry_counter = 0
    self.v25k_brake_release_counter = 0
    self.v25k_propulsion_block_until_frame = 0
    self.v25k_accel_entry_counter = 0
    self.v25k_speed_offset = 0.0

  def update(self, enabled, active, CS, frame, actuators, pcm_cancel_cmd,
             hud_alert, left_line, right_line, lead,
             left_lane_depart, right_lane_depart, dragonconf):

    can_sends = []  # Create the list that will hold all CAN messages for this control cycle

    # -----------------------------
    # Steering
    # -----------------------------
    steer_max_interp = interp(CS.out.vEgo, self.params.STEER_BP, self.params.STEER_LIM_TORQ)  # Get speed-based steering torque limit
    steer_max_interp = max(1.0, steer_max_interp)  # Avoid divide-by-zero if the torque table returns 0

    new_steer = int(round(actuators.steer * steer_max_interp))  # Convert normalized OP steer command into raw torque

    isBlinkerOn = CS.out.leftBlinker != CS.out.rightBlinker  # True when only one blinker is active

    apply_steer = apply_dnga_steer_torque_limits(  # Apply steering torque and rate limits
      new_steer,  # Requested raw steering torque
      self.last_steer,  # Last applied raw steering torque
      CS.out.steeringTorqueEps,  # Driver torque from EPS
      isBlinkerOn,  # Blinker state for driver override allowance
      self.params  # Steering limit parameters
    )

    self.steer_rate_limited = (new_steer != apply_steer) and (apply_steer != 0)  # Mark whether steering was rate limited
    self.steer_rate_limited &= not CS.out.steeringPressed  # Do not show rate limit if driver is steering

    # -----------------------------
    # Longitudinal base values
    # -----------------------------
    apply_accel = clip(actuators.accel, -3.0, 1.5)  # Clamp openpilot accel request
    apply_brake = -apply_accel if apply_accel < 0.0 else 0.0  # Convert negative accel into positive brake amount

    if enabled and not self.prev_enabled:  # Detect the first frame after OP engagement
      # Thermald now gates startup until ignition is stable. Retain only a
      # short hydraulic settle time so engagement behind a lead does not
      # create a two-second late-brake catch-up.
      self.block_brake_until_frame = frame + 50
      self.v25k_apply_brake = 0.0
      self.v25k_brake_active = False
      self.v25k_brake_entry_counter = 0
      self.v25k_brake_release_counter = 0
      self.v25k_accel_entry_counter = 0
      self.v25k_speed_offset = 0.0

    self.prev_enabled = enabled  # Save enabled state for the next control cycle

    # Do not send diagnostic clear-code frames automatically at startup.
    if self.need_clear_engine:
      can_sends.append(make_can_msg(2015, b'\x01\x04\x00\x00\x00\x00\x00\x00', 0))

    # -----------------------------
    # Steering command, 50 Hz
    # -----------------------------
    if (frame % 2) == 0:  # Send steering every 2 frames
      steer_req = (enabled or self.stockLdw) and CS.lkas_latch  # Request steering when OP/LDA is active and LKAS latch is on
      can_sends.append(  # Add steering CAN message
        create_can_steer_command(  # Build STEERING_LKAS frame
          self.packer,  # CAN packer
          apply_steer,  # Raw torque command
          steer_req,  # Steering request bit
          (frame // 2) % 16  # Steering counter
        )
      )

    # -----------------------------
    # Longitudinal / HUD, 20 Hz
    # -----------------------------
    if (frame % 5) == 0:  # Send ACC/HUD messages every 5 frames

      # -----------------------------
      # ACC_BRAKE / 0x271 state map from logs
      # -----------------------------
      # 0x00 + pump 0.0 + mag 200 = disabled / neutral / no brake
      # 0x01 + pump 0.0 + mag 200 = enabled / ready / no brake
      # 0x21 + pump -0.4 + mag 0x04xx = active braking request
      # 0x30 and 0x31 are likely stop-hold states; do not use them yet

      if not enabled:  # OP is not engaged
        brake_state = 0x00  # Use disabled/neutral brake state
        pump_reaction = 0.0  # No pump reaction
        brake_mag = 200  # Stock neutral magnitude 0x00C8

      else:  # OP is engaged but not necessarily braking
        brake_state = 0x01  # Use enabled/no-brake state found in logs
        pump_reaction = 0.0  # No pump reaction while not braking
        brake_mag = 200  # Stock neutral magnitude 0x00C8

      # -----------------------------
      # V2.5K stock-region hydraulic braking
      # -----------------------------
      # Small negative requests stay in the ACC_CMD/regen path. Once hydraulic
      # braking enters, brief planner sign reversals ramp toward the known
      # minimum instead of instantly changing 0x21 back to 0x01.
      brake_request = apply_brake

      hard_brake_release = (
        not enabled or
        not CS.out.cruiseState.enabled or
        pcm_cancel_cmd or
        CS.out.gasPressed or
        CS.out.brakePressed
      )

      moving_brake_allowed = (
        enabled and
        CS.out.cruiseState.enabled and
        not CS.out.standstill and
        frame > self.block_brake_until_frame and
        CS.out.vEgo > V25K_MIN_MOVING_SPEED and
        not CS.out.gasPressed and
        not CS.out.brakePressed
      )

      if hard_brake_release or not moving_brake_allowed:
        if self.v25k_brake_active:
          self.v25k_propulsion_block_until_frame = max(
            self.v25k_propulsion_block_until_frame,
            frame + V25K_PROPULSION_DWELL_FRAMES,
          )
        self.v25k_apply_brake = 0.0
        self.v25k_brake_active = False
        self.v25k_brake_entry_counter = 0
        self.v25k_brake_release_counter = 0
      else:
        if not self.v25k_brake_active:
          if brake_request >= V25K_BRAKE_ENTRY:
            self.v25k_brake_entry_counter = min(
              self.v25k_brake_entry_counter + 1,
              V25K_BRAKE_ENTRY_FRAMES,
            )
          else:
            self.v25k_brake_entry_counter = 0

          if (
            brake_request >= V25K_BRAKE_URGENT_ENTRY or
            self.v25k_brake_entry_counter >= V25K_BRAKE_ENTRY_FRAMES
          ):
            self.v25k_brake_active = True
            self.v25k_brake_release_counter = 0

        if self.v25k_brake_active:
          if brake_request < V25K_BRAKE_MIN:
            self.v25k_brake_release_counter = min(
              self.v25k_brake_release_counter + 1,
              V25K_BRAKE_RELEASE_FRAMES,
            )
          else:
            self.v25k_brake_release_counter = 0

          low_speed_cap = v25k_low_speed_brake_cap(CS.out.vEgo)
          target_brake = (
            float(clip(
              brake_request,
              V25K_BRAKE_MIN,
              min(V25K_BRAKE_MAX, low_speed_cap),
            ))
            if brake_request >= V25K_BRAKE_MIN
            else V25K_BRAKE_MIN
          )

          self.v25k_apply_brake = max(
            V25K_BRAKE_MIN,
            v25k_rate_limit_brake(
              target_brake,
              self.v25k_apply_brake,
            ),
          )

          # Keep propulsion neutral throughout the hydraulic episode and for
          # a short dwell after release. This prevents direct brake/accel
          # alternation even when the planner crosses zero repeatedly.
          self.v25k_propulsion_block_until_frame = max(
            self.v25k_propulsion_block_until_frame,
            frame + V25K_PROPULSION_DWELL_FRAMES,
          )

          if (
            self.v25k_brake_release_counter >= V25K_BRAKE_RELEASE_FRAMES and
            self.v25k_apply_brake <= V25K_BRAKE_MIN
          ):
            self.v25k_apply_brake = 0.0
            self.v25k_brake_active = False
            self.v25k_brake_entry_counter = 0
            self.v25k_brake_release_counter = 0

      decel_req = self.v25k_brake_active and self.v25k_apply_brake > 0.0

      if decel_req:
        brake_state = 0x21
        pump_reaction, brake_mag = v25k_encode_hev_brake(
          self.v25k_apply_brake
        )

      # -----------------------------
      # V2.5K regen/acceleration handoff
      # -----------------------------
      # Keep a desired-speed offset relative to current speed. Unlike V2.5I,
      # it does not jump a negative ACC_CMD straight back to vEgo on the first
      # positive planner sample.
      t_lookup = 0.35 + 0.07 * CS.out.vEgo
      propulsion_blocked = (
        decel_req or
        frame < self.v25k_propulsion_block_until_frame or
        CS.out.aEgo < -0.12
      )

      if (
        not enabled or
        not CS.out.cruiseState.enabled or
        pcm_cancel_cmd or
        CS.out.standstill or
        CS.out.gasPressed or
        CS.out.brakePressed
      ):
        self.v25k_accel_entry_counter = 0
        self.v25k_speed_offset = 0.0
      else:
        if apply_accel >= V25K_ACCEL_ENTRY and not propulsion_blocked:
          self.v25k_accel_entry_counter = min(
            self.v25k_accel_entry_counter + 1,
            V25K_ACCEL_ENTRY_FRAMES,
          )
        else:
          self.v25k_accel_entry_counter = 0

        if apply_accel < -V25K_BRAKE_MIN:
          target_speed_offset = max(
            -V25K_SPEED_OFFSET_NEG_MAX,
            apply_accel * t_lookup,
          )
        elif (
          self.v25k_accel_entry_counter >= V25K_ACCEL_ENTRY_FRAMES and
          not propulsion_blocked
        ):
          # Limit lead-recovery acceleration more at low speed, where the
          # hybrid drivetrain response was reported as most abrupt.
          positive_accel_cap = float(interp(
            CS.out.vEgo,
            [0.0, 5.0, 15.0],
            [0.18, 0.25, 0.30],
          ))
          target_speed_offset = (
            min(apply_accel, positive_accel_cap) * t_lookup
          )
        else:
          target_speed_offset = 0.0

        if propulsion_blocked and self.v25k_speed_offset > 0.0:
          self.v25k_speed_offset = 0.0

        self.v25k_speed_offset = v25k_update_speed_offset(
          target_speed_offset,
          self.v25k_speed_offset,
        )

      des_speed = max(0.0, CS.out.vEgo + self.v25k_speed_offset)

      # Stock 0x273 occasionally requests deceleration while 0x271 remains in
      # ready state. Preserve that regen/coast path for small negative demand.
      decel_mode = (
        enabled and
        not CS.out.standstill and
        (
          decel_req or
          self.v25k_speed_offset < -0.01 or
          apply_accel < -V25K_BRAKE_MIN
        )
      )
      brake_amt_for_hud = (
        self.v25k_apply_brake
        if decel_req
        else (V25K_BRAKE_MIN if decel_mode else 0.0)
      )

      can_sends.append(  # Add ACC_CMD_HUD message
        dnga_create_accel_command(  # Build 0x273 ACC command frame
          self.packer,  # CAN packer
          CS.cruise_speed,  # OP cruise set speed
          CS.out.cruiseState.available,  # ACC ready/available bit
          enabled,  # OP enabled state
          lead,  # Lead visible flag
          des_speed,  # Desired speed command
          brake_amt_for_hud,  # Brake amount used for IS_DECEL/IS_ACCEL
          CS.op_distance_val  # Follow distance setting
        )
      )

      can_sends.append(  # Add ACC_BRAKE message
        dnga_create_brake_command(  # Build 0x271 brake command
          self.packer,  # CAN packer
          brake_state,  # 0x00, 0x01, or 0x21
          pump_reaction,  # 0.0 or -0.4
          brake_mag,  # 200 or conservative 0x04xx value
          (frame // 5) % 8  # ACC_BRAKE counter
        )
      )

      can_sends.append(  # Add LKAS_HUD message
        dnga_create_hud(  # Build 0x274 HUD command
          self.packer,  # CAN packer
          CS.out.cruiseState.available and CS.lkas_latch,  # LKAS ready state
          enabled,  # LKAS engaged display state
          left_line,  # Left lane visible
          right_line,  # Right lane visible
          self.stockLdw,  # Lane departure warning flag
          CS.stock_fcw,  # Stock FCW state
          CS.stock_aeb,  # Stock AEB state
          CS.stock_adas_frontDepartureHUD,  # Front departure HUD state
          CS.stock_lkc_off,  # LKC off state
          CS.stock_fcw_off  # FCW off state
        )
      )

    self.last_steer = apply_steer  # Store applied steering torque for next frame

    new_actuators = actuators.copy()  # Copy actuator output object
    new_actuators.steer = apply_steer / steer_max_interp  # Report actual normalized steering command

    return new_actuators, can_sends  # Return actuator feedback and CAN messages
