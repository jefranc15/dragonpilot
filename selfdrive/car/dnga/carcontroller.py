from selfdrive.car import make_can_msg  # Helper for creating raw CAN messages
from selfdrive.car.dnga.dngacan import create_can_steer_command, \  # LKAS steering CAN command
                                       dnga_create_accel_command, \  # ACC_CMD_HUD / acceleration command
                                       dnga_create_brake_command, \  # ACC_BRAKE / braking command
                                       dnga_create_hud  # LKAS_HUD / cluster HUD command
from selfdrive.car.dnga.values import DBC, NOT_CAN_CONTROLLED  # Car DBC map and steering-control car set
from opendbc.can.packer import CANPacker  # Packs signal dictionaries into CAN frames
from common.numpy_fast import clip, interp  # Fast clamp and interpolation helpers
from selfdrive.config import Conversions as CV  # Unit conversions such as KPH_TO_MS

try:  # Try to import dragonpilot custom feature flags
  from common.features import Features  # Feature flag helper
except ImportError:  # If the feature module does not exist
  class Features:  # Create a safe fallback class
    def has(self, feature_name):  # Fallback feature lookup
      return False  # Default all optional features to disabled


def apply_dnga_steer_torque_limits(apply_torque, apply_torque_last, driver_torque, blinkerOn, LIMITS):  # Limit steering torque command safely
  reduced_torque_mult = 10 if blinkerOn else 1.5  # Allow more driver/blinker override when signal is on

  driver_max_torque = 255 + driver_torque * reduced_torque_mult  # Upper steering limit based on driver torque
  driver_min_torque = -255 - driver_torque * reduced_torque_mult  # Lower steering limit based on driver torque

  max_steer_allowed = clip(driver_max_torque, 0, 255)  # Clamp max torque to allowed positive range
  min_steer_allowed = clip(driver_min_torque, -255, 0)  # Clamp min torque to allowed negative range

  apply_torque = clip(apply_torque, min_steer_allowed, max_steer_allowed)  # Apply driver torque limit

  if apply_torque_last > 0:  # If last command was positive
    apply_torque = clip(  # Apply rate limit while steering positive
      apply_torque,  # Requested torque
      max(apply_torque_last - LIMITS.STEER_DELTA_DOWN, -LIMITS.STEER_DELTA_UP),  # Lower bound
      apply_torque_last + LIMITS.STEER_DELTA_UP  # Upper bound
    )
  else:  # If last command was zero or negative
    apply_torque = clip(  # Apply rate limit while steering negative
      apply_torque,  # Requested torque
      apply_torque_last - LIMITS.STEER_DELTA_UP,  # Lower bound
      min(apply_torque_last + LIMITS.STEER_DELTA_DOWN, LIMITS.STEER_DELTA_UP)  # Upper bound
    )

  return int(round(float(apply_torque)))  # Return integer torque command


class CarControllerParams():  # Container for controller tuning constants
  def __init__(self, CP):  # Initialize using CarParams
    self.STEER_BP = CP.lateralParams.torqueBP  # Speed breakpoints for steering torque limit
    self.STEER_LIM_TORQ = CP.lateralParams.torqueV  # Steering torque values for each breakpoint

    if CP.carFingerprint in NOT_CAN_CONTROLLED:  # If this car is not CAN controlled
      self.STEER_DELTA_UP = 20  # Faster torque increase for non-CAN-controlled case
      self.STEER_DELTA_DOWN = 30  # Torque decrease limit
    else:  # Normal DNGA CAN steering case
      self.STEER_DELTA_UP = 17  # Torque increase limit
      self.STEER_DELTA_DOWN = 30  # Torque decrease limit


class CarController():  # Main car controller class
  def __init__(self, dbc_name, CP, VM):  # Initialize controller
    self.last_steer = 0  # Store last applied steering torque
    self.steer_rate_limited = False  # Track whether steering was rate limited

    self.params = CarControllerParams(CP)  # Load controller parameters
    self.packer = CANPacker(DBC[CP.carFingerprint]['pt'])  # Create CAN packer using powertrain DBC

    f = Features()  # Create feature flag helper
    self.need_clear_engine = f.has("ClearCode")  # Optional diagnostic clear-code feature

    self.stockLdw = False  # Stock lane-departure warning flag placeholder

    self.prev_enabled = False  # Previous openpilot enabled state
    self.block_brake_until_frame = 0  # Frame until which braking is blocked after engagement

  def update(self, enabled, active, CS, frame, actuators, pcm_cancel_cmd,
             hud_alert, left_line, right_line, lead,
             left_lane_depart, right_lane_depart, dragonconf):  # Main update loop called by interface

    can_sends = []  # List of CAN messages to send this frame

    # -----------------------------
    # Steering
    # -----------------------------
    steer_max_interp = interp(CS.out.vEgo, self.params.STEER_BP, self.params.STEER_LIM_TORQ)  # Get speed-based steering max
    steer_max_interp = max(1.0, steer_max_interp)  # Prevent divide-by-zero later

    new_steer = int(round(actuators.steer * steer_max_interp))  # Convert normalized steer command to raw torque

    isBlinkerOn = CS.out.leftBlinker != CS.out.rightBlinker  # True if exactly one blinker is active

    apply_steer = apply_dnga_steer_torque_limits(  # Apply steering torque safety limits
      new_steer,  # Requested steering torque
      self.last_steer,  # Previous steering torque
      CS.out.steeringTorqueEps,  # Driver/EPS torque reading
      isBlinkerOn,  # Blinker state
      self.params  # Steering limit parameters
    )

    self.steer_rate_limited = (new_steer != apply_steer) and (apply_steer != 0)  # Mark steering as limited if changed
    self.steer_rate_limited &= not CS.out.steeringPressed  # Do not alert rate limit if driver is steering

    # -----------------------------
    # Longitudinal base values
    # -----------------------------
    apply_accel = clip(actuators.accel, -3.0, 1.5)  # Clamp OP requested accel/decel
    apply_brake = -apply_accel if apply_accel < 0.0 else 0.0  # Convert negative accel into positive brake request

    if enabled and not self.prev_enabled:  # If OP just became enabled
      self.block_brake_until_frame = frame + 200  # Block brake state for about 2 seconds after SET/engage

    self.prev_enabled = enabled  # Save enabled state for next loop

    if self.need_clear_engine or frame < 1000:  # If clear-code feature is on or device just booted
      can_sends.append(make_can_msg(2015, b'\x01\x04\x00\x00\x00\x00\x00\x00', 0))  # Send diagnostic clear frame

    # -----------------------------
    # Steering command, 50 Hz
    # -----------------------------
    if (frame % 2) == 0:  # Send steering every 2 frames
      steer_req = (enabled or self.stockLdw) and CS.lkas_latch  # Request steering only when enabled/LDA and LKAS latch is on
      can_sends.append(  # Add steering CAN message
        create_can_steer_command(  # Build STEERING_LKAS frame
          self.packer,  # CAN packer
          apply_steer,  # Raw torque command
          steer_req,  # Steering request bit
          (frame // 2) % 16  # 4-bit counter
        )
      )

    # -----------------------------
    # Longitudinal / HUD, 20 Hz
    # -----------------------------
    if (frame % 5) == 0:  # Send longitudinal and HUD messages every 5 frames

      boost = interp(CS.out.vEgo, [0.2, 0.5, 18.0, 23.0], [0.0, 1.0, 1.0, 1.0])  # Reduce accel command at very low speed
      base_speed = getattr(actuators, 'speed', CS.out.vEgo)  # Use actuator target speed if available, otherwise current speed
      des_speed = max(0.0, base_speed + clip(actuators.accel * boost, -1.0, 1.0))  # Desired ACC command speed

      # -----------------------------
      # ACC_BRAKE / 0x271 state map from your logs
      # -----------------------------
      # 0x00 + pump 0.0 + mag 200 = disabled / neutral / no brake
      # 0x01 + pump 0.0 + mag 200 = enabled / ready / no brake
      # 0x21 + pump -0.4 + mag 0x04xx = active braking request
      # Do not use 0x30 or 0x31 yet; those are likely stop-hold states.

      if not enabled:  # If OP is not enabled
        brake_state = 0x00  # Send neutral disabled brake state
        pump_reaction = 0.0  # No pump reaction
        brake_mag = 200  # Stock neutral magnitude 0x00C8

      else:  # If OP is enabled
        brake_state = 0x01  # Send enabled/no-brake state found in logs
        pump_reaction = 0.0  # No pump reaction while not braking
        brake_mag = 200  # Stock neutral magnitude 0x00C8

      decel_req = (  # Decide if OP is truly requesting moving braking
        enabled and  # OP must be enabled
        frame > self.block_brake_until_frame and  # Do not brake immediately after SET
        CS.out.vEgo > 20.0 * CV.KPH_TO_MS and  # Do not use OP brake below 20 kph yet
        apply_brake > 0.20 and  # Require meaningful decel request
        apply_brake < 0.70 and  # Ignore strong decel request for first safety test
        not CS.out.gasPressed and  # Do not brake if driver is pressing gas
        not CS.out.brakePressed  # Do not send OP brake if driver is pressing brake
      )

      if decel_req:  # If OP actually wants mild moving decel
        brake_state = 0x21  # Active brake request state from stock logs
        pump_reaction = -0.4  # Pump value seen in stock 0x21 frames
        brake_mag = int(interp(  # Map OP brake request to conservative stock-like magnitude
          clip(apply_brake, 0.20, 0.70),  # Clamp brake request into test range
          [0.20, 0.70],  # OP brake request range
          [1215, 1205]  # Very narrow weak-braking test range near stock 0x04xx values
        ))

      brake_amt_for_hud = apply_brake if decel_req else 0.0  # Tell ACC_CMD_HUD decel only when sending 0x21

      can_sends.append(  # Add ACC_CMD_HUD message
        dnga_create_accel_command(  # Build 0x273 ACC command frame
          self.packer,  # CAN packer
          CS.cruise_speed,  # OP cruise set speed
          CS.out.cruiseState.available,  # ACC ready/available bit used by HUD command
          enabled,  # OP enabled state
          lead,  # Lead visible flag
          des_speed,  # Desired speed command
          brake_amt_for_hud,  # Brake amount for IS_DECEL/IS_ACCEL selection
          CS.op_distance_val  # Follow distance value
        )
      )

      can_sends.append(  # Add ACC_BRAKE message
        dnga_create_brake_command(  # Build 0x271 brake command
          self.packer,  # CAN packer
          brake_state,  # 0x00, 0x01, or 0x21
          pump_reaction,  # 0.0 or -0.4
          brake_mag,  # 200 or conservative 0x04xx value
          (frame // 5) % 8  # 3-bit counter
        )
      )

      can_sends.append(  # Add LKAS_HUD message
        dnga_create_hud(  # Build 0x274 HUD command
          self.packer,  # CAN packer
          CS.out.cruiseState.available and CS.lkas_latch,  # LKAS ready
          enabled,  # LKAS engaged visual state
          left_line,  # Left lane visible
          right_line,  # Right lane visible
          self.stockLdw,  # Lane departure warning
          CS.stock_fcw,  # Stock forward collision warning
          CS.stock_aeb,  # Stock AEB state
          CS.stock_adas_frontDepartureHUD,  # Front departure HUD state
          CS.stock_lkc_off,  # LKC off state
          CS.stock_fcw_off  # FCW off state
        )
      )

    self.last_steer = apply_steer  # Save steering command for next frame

    new_actuators = actuators.copy()  # Copy actuator object for return
    new_actuators.steer = apply_steer / steer_max_interp  # Report applied normalized steer value

    return new_actuators, can_sends  # Return applied actuators and CAN messages
