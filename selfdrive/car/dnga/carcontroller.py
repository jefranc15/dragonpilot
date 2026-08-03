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
from selfdrive.config import Conversions as CV
from cereal import messaging


try:
  from common.features import Features
except ImportError:
  class Features:
    def has(self, feature_name):
      return False


# V3.3R: V3.0/V3.2R baseline with stock-derived release, hold, and softer highway braking.
#
# V2.5R keeps V2.5P/Q's explicit stock-observed 0x273 state mapping and
# 0x21 -> 0x31 -> 0x30 stop-and-go states, but changes command arbitration:
#
#   * A fresh longitudinalPlan is the primary direction request. A temporary
#     positive downstream PID correction cannot release hydraulic braking or
#     request propulsion while the planner still asks to slow for a lead.
#   * Normal lead following uses 0x01 coasting/regen first. Hydraulic braking
#     starts weak and progressively builds only after sustained planner decel.
#   * Normal lead hydraulic authority is intentionally below the observed stock
#     maximum. A separate urgent path keeps enough authority for a genuinely
#     fast closing event.
#   * After braking, lead propulsion requires sustained positive planner demand
#     and remains capped to a low rate, preventing rev/jump/brake cycling.
#   * The selected 1/2/3-bar value changes local hydraulic sensitivity.
#     carstate.py now publishes the same selection through distanceLines, so
#     the MPC uses the matching aggressive/standard/relaxed time gap.
#
# Steering limits and interface longitudinal gains are deliberately unchanged
# in this patch. They should be changed separately only after the longitudinal
# actuator handoff is stable.
V25L_BRAKE_MIN = 0.05

# Progressive pedal-like shaping at the 20 Hz longitudinal CAN rate.
V25R_BRAKE_STEP_UP = 0.015
V25R_BRAKE_STEP_UP_URGENT = 0.030
V25R_BRAKE_STEP_DOWN = 0.015
V25R_BRAKE_FILTER_UP = 0.08
V25R_BRAKE_FILTER_DOWN = 0.08

V25R_LEAD_ENTRY_FRAMES = 4          # 0.20 seconds
V25R_URGENT_ENTRY_FRAMES = 2        # 0.10 seconds
V25L_LEAD_TRUST_FRAMES = 10         # 0.50 seconds
V25L_URGENT_LEAD_FRAMES = 3         # 0.15 seconds
V25L_REENTRY_BLOCK_FRAMES = 80      # 0.80 seconds at 100 Hz
V25L_PROPULSION_DWELL_FRAMES = 60   # 0.60 seconds after hydraulic release
V25L_MIN_MOVING_SPEED = 0.15
V25L_MIN_ENTRY_SPEED = 0.50

# Planner/radar freshness and hysteresis.
V25R_PLAN_MAX_AGE_FRAMES = 50       # 0.50 seconds at 100 Hz
V25R_RADAR_MAX_AGE_FRAMES = 50
V25R_RELEASE_CONFIRM_FRAMES = 10    # 0.50 seconds at 20 Hz
V25R_LEAD_ACCEL_CONFIRM_FRAMES = 3  # 0.15 seconds at 20 Hz
V25R_LEAD_LOSS_FRAMES = 3           # ignore one 20 Hz lead dropout
V25R_PLANNER_DECEL_DEADBAND = 0.03
V25R_PLANNER_ACCEL_ENTRY = 0.05
V25R_PLANNER_RELEASE = 0.05

# Normal lead braking is deliberately gentler than stock. Urgent closing can
# still use more authority, but remains below V2.5R's 0.75 m/s^2 maximum.
V25R_URGENT_PLANNER_DECEL = 0.45
V25R_URGENT_HARD_DECEL = 0.70
V25R_URGENT_CLOSING_SPEED = 2.5     # m/s
V25R_URGENT_TTC = 7.0               # seconds
V25R_URGENT_BRAKE_MAX = 0.45

# Planner-primary target with only a small same-direction PID allowance.
V25R_PID_BRAKE_BLEND = 0.25
V25R_PID_BRAKE_ALLOWANCE = 0.08

# V3.0 early-progressive highway lead braking.
V30_HIGHWAY_MIN_SPEED = 8.0
V30_NORMAL_ENTRY_MIN = 0.12
V30_NORMAL_ENTRY_MAX = 0.20
V30_NORMAL_BRAKE_CAP_MIN = 0.25
V30_NORMAL_BRAKE_CAP_MAX = 0.30
V30_BRAKE_STEP_UP = 0.006
V30_BRAKE_STEP_UP_URGENT = 0.015
V30_HANDOFF_PID_ACCEL = 0.15
V30_HANDOFF_AEGO_MAX = -0.08
V30_HANDOFF_FRAMES = 3
V30_HANDOFF_STEP_DOWN = 0.030
V30_REENTRY_BLOCK_FRAMES = 30

# V3.2R isolated low-speed ECU wake/handoff.
#
# This is intentionally separate from the V3.0 highway and curve logic:
#   * V3.0 hydraulic entry, caps, handoff and curve agreement stay unchanged.
#   * The outgoing fake lead bit is allowed only below 30.6 km/h.
#   * It first arms at exactly zero desired-speed offset.
#   * Positive target still requires measured deceleration to fade, retaining
#     V3.0's -0.10 m/s² aEgo gate instead of pushing through active regen.
V32R_LOW_SPEED_MAX = 8.5
V32R_REGEN_NEUTRAL_DWELL_FRAMES = 50   # 0.50 s at 100 Hz
V32R_LOW_SPEED_ARM_FRAMES = 30         # 0.30 s lead-bit-only neutral stage
V32R_LOW_SPEED_ACCEL_CAP = 0.10
V32R_LOW_SPEED_OFFSET_CAP = 0.18
V32R_LOW_SPEED_OFFSET_STEP_UP = 0.003
V32R_DEPARTING_LEAD_VREL = 0.40
V32R_DEPARTING_LEAD_DREL = 3.0
V32R_DEPARTING_ACCEL_CAP = 0.12
V32R_DEPARTING_OFFSET_CAP = 0.20
V32R_DEPARTING_OFFSET_STEP_UP = 0.004
V32R_NONBLOCKING_LEAD_DISTANCE = 20.0
V32R_NONBLOCKING_LEAD_VREL = -0.50
V32R_LOW_SPEED_OVERSHOOT_AEGO = 0.45
V32R_OVERSHOOT_BLOCK_FRAMES = 50

# V3.3R stock-derived safety layers.
#
# The confirmed engine RPM signal is diagnostic only. These safeguards use
# planner direction, target slope, hydraulic state, trusted lead motion, and
# actual vehicle acceleration because the logged launch happened at 0 RPM.
V33R_LOW_SPEED_ENGAGEMENT_MAX = 4.17
V33R_LOW_SPEED_ENGAGEMENT_GUARD_FRAMES = 50

V33R_STOP_LEAD_TRUST_FRAMES = 3
V33R_STOP_HOLD_MAX_EGO = 0.15
V33R_STOP_HOLD_MIN_DISTANCE = 1.0
V33R_STOP_HOLD_MAX_DISTANCE = 12.0
V33R_STOPPED_LEAD_MAX_SPEED = 0.50
V33R_HOLD_RESUME_LEAD_SPEED = 0.35
V33R_HOLD_RESUME_FRAMES = 6
V33R_CREEP_GUARD_MAX_EGO = 1.00
V33R_CREEP_GUARD_MIN_CLOSING = 0.05
V33R_CREEP_ENTRY_FRAMES = 2
V33R_CREEP_BRAKE_FLOOR = 0.18

V33R_RELEASE_FREEZE_FRAMES = 30
V33R_RELEASE_LEAD_HOLD_FRAMES = 80
V33R_RELEASE_PROPULSION_BLOCK_FRAMES = 60
V33R_TARGET_SLOPE_UNLOCK_FRAMES = 30
V33R_TARGET_RETURN_STEP = 0.010
V33R_TARGET_SLOPE_BRAKE = 0.02
V33R_TARGET_SLOPE_AEGO = -0.08
V33R_PROPULSION_AEGO_MIN = -0.05

V33R_OVERSHOOT_AEGO = 0.35
V33R_OVERSHOOT_CONFIRM_FRAMES = 2
V33R_OVERSHOOT_BLOCK_FRAMES = 60

V33R_EARLY_HIGHWAY_MIN_SPEED = 8.0
V33R_EARLY_HIGHWAY_CLOSING = 1.0
V33R_EARLY_HIGHWAY_TTC = 18.0
V33R_EARLY_HIGHWAY_TIME_GAP = 3.0
V33R_EARLY_HIGHWAY_PLANNER_BRAKE = 0.05

V33R_AEGO_FILTER_ALPHA = 0.20
V33R_DECEL_GOVERNOR_START = -0.90
V33R_DECEL_GOVERNOR_CRITICAL_TTC = 3.0
V33R_DECEL_GOVERNOR_CRITICAL_CLOSING = 3.0
V33R_DECEL_GOVERNOR_STEP_DOWN = 0.015

# Stock-like 0x01 desired-speed shaping. Positive acceleration follows the
# previously successful continuous target strategy, while negative planner
# demand ramps into a conservative current-speed-relative offset. This allows
# no-lead curve slowing without enabling hydraulic braking.
V25L_DECEL_DEADBAND = 0.02
V25L_DECEL_OFFSET_STEP_DOWN = 0.02
V25L_DECEL_OFFSET_STEP_UP = 0.08
V25L_ACCEL_ENTRY = 0.05
V25L_ACCEL_CAP = 0.25
V25L_ACCEL_OFFSET_STEP_UP = 0.004
V25L_ACCEL_OFFSET_STEP_DOWN = 0.04

# V2.5V regen-to-propulsion neutral handoff. The DNGA 0x273 normal mode
# covers both lowered-speed regen and positive propulsion, so prevent the
# requested speed from crossing directly from below vEgo to above vEgo.
V25V_REGEN_RELEASE_DWELL_FRAMES = 30  # 0.30 seconds at 100 Hz
V25V_REGEN_OFFSET_EPS = 0.01          # m/s
V25V_ACCEL_AEGO_MIN = -0.10           # wait until measured decel has faded
V25L_ACCEL_OFFSET_MAX = 0.55

# V2.5O mild no-lead hydraulic supplement. It requires a sustained strong
# planner request, but the sent brake command is intentionally much smaller.
V25O_NOLEAD_BRAKE_ENTRY = 0.40
V25O_NOLEAD_BRAKE_RELEASE = 0.16
V25O_NOLEAD_BRAKE_MAX = 0.10
V25O_NOLEAD_ENTRY_FRAMES = 10
V25O_NOLEAD_MIN_SPEED = 3.0        # 10.8 km/h; never stop the car without a lead

# V2.5W curve-only hydraulic supplement. Entry requires the longitudinal
# planner itself to select source="turn". Generic no-lead braking remains at
# 0.10 m/s^2, so hills, speed corrections, and ordinary no-lead cruise do not
# automatically receive the stronger curve authority.
V25W_CURVE_BRAKE_ENTRY = 0.16
V25W_CURVE_BRAKE_RELEASE = 0.08
V25W_CURVE_ENTRY_FRAMES = 3       # 0.15 seconds at 20 Hz
V25W_CURVE_PID_ENTRY = 0.25

# Stock-observed stop-and-go states. 0x31 is used during the final crawl and
# 0x30 holds standstill at the minimum known brake command (0x04C3).
V25O_SNG_ARM_SPEED = 2.5            # 9.0 km/h
V25O_SNG_APPROACH_SPEED = 1.2       # 4.3 km/h
V25O_SNG_ARM_BRAKE = 0.18
V25O_SNG_HOLD_BRAKE = 0.05
V25O_SNG_RELEASE_ACCEL = 0.08
V25O_SNG_RELEASE_FRAMES = 5         # 0.25 seconds at 20 Hz

# V2.5U low-speed stopped-lead protection. This bypasses only the normal
# hydraulic reentry delay when the vehicle is already crawling toward a
# nearly stopped lead at close range.
V25U_STOP_LEAD_MAX_SPEED = 0.50       # estimated lead speed, m/s
V25U_STOP_LEAD_MAX_DISTANCE = 8.0     # metres
V25U_STOP_LEAD_MIN_CLOSING = 0.15     # m/s
V25U_STOP_LEAD_MIN_BRAKE = 0.08       # planner decel, m/s^2

# V2.5X-SC1 stop-completion-only correction.
#
# Once the existing stop-and-go path is armed and already in the final crawl
# toward a trusted stopped lead, do not let the hydraulic command taper below
# the level that failed to overcome vehicle creep in the 2026-07-31 log.
#
# These constants do not change hydraulic entry, following distance, regen,
# acceleration, resume, lead trust, or the standstill/0x30 transition.
V25X_SC_MAX_EGO_SPEED = 0.80           # 2.9 km/h: final crawl only
V25X_SC_MAX_LEAD_SPEED = 0.50          # lead is effectively stopped
V25X_SC_MAX_LEAD_DISTANCE = 8.0        # retain existing stopped-lead limit
V25X_SC_MIN_BRAKE = 0.18               # m/s^2 hydraulic completion floor

V25O_BRAKE_MODE_NONE = 0
V25O_BRAKE_MODE_LEAD = 1
V25O_BRAKE_MODE_NOLEAD = 2
V25W_BRAKE_MODE_CURVE = 3


def v25r_rate_limit_brake(target, last, urgent=False):
  step_up = V25R_BRAKE_STEP_UP_URGENT if urgent else V25R_BRAKE_STEP_UP
  if target > last:
    return min(target, last + step_up)
  return max(target, last - V25R_BRAKE_STEP_DOWN)


def v25r_normalize_plan_source(source):
  """Return a stable lowercase longitudinalPlan source name."""
  try:
    source = str(source).strip().lower()
  except Exception:
    return ""
  if "." in source:
    source = source.rsplit(".", 1)[-1]
  return source


def v25r_distance_profile(distance_val):
  """Return planner-entry, normal brake cap, and lead accel cap.

  CS.op_distance_val follows the local SetDistance enum:
    0 = aggressive / 1 bar
    1 = normal / 2 bars
    2 = far / 3 bars
  """
  try:
    distance_val = int(distance_val)
  except Exception:
    distance_val = 1

  # V2.5X evenly scaled bar profile:
  #   1 bar: closest, latest hydraulic entry, strongest normal authority
  #   2 bars: balanced midpoint
  #   3 bars: farthest, slightly earlier entry, gentlest normal authority
  #
  # Keep the current 3-bar hydraulic threshold at 0.16 so the previous
  # too-early 3-bar braking does not return. Scale 1 and 2 bars around it.
  if distance_val <= 0:
    return 0.18, 0.42, 0.12
  if distance_val >= 2:
    return 0.16, 0.36, 0.08
  return 0.17, 0.39, 0.10


def v25l_low_speed_brake_cap(v_ego):
  """Taper moving brake authority smoothly toward zero vehicle speed."""
  return float(interp(
    v_ego,
    [0.15, 0.30, 0.60, 1.00, 1.50, 2.50],
    [0.05, 0.08, 0.15, 0.25, 0.40, 0.75],
  ))


def v25l_high_speed_brake_cap(v_ego):
  """Keep hydraulic authority conservative at road/highway speed."""
  return float(interp(
    v_ego,
    [0.0, 15.0, 25.0, 35.0, 45.0],
    [0.65, 0.65, 0.60, 0.52, 0.48],
  ))


def v25w_curve_brake_cap(v_ego):
  """Moderate curve-only hydraulic authority selected by planner source."""
  return float(interp(
    v_ego,
    [3.0, 8.0, 15.0, 25.0, 35.0],
    [0.12, 0.17, 0.23, 0.28, 0.30],
  ))


def v25l_hydraulic_entry(v_ego):
  """Legacy speed-dependent hydraulic entry curve."""
  return float(interp(
    v_ego,
    [0.0, 4.0, 15.0, 25.0, 35.0],
    [0.25, 0.25, 0.38, 0.47, 0.52],
  ))


def v30_progressive_hydraulic_entry(v_ego):
  return float(interp(
    v_ego,
    [8.0, 15.0, 25.0, 35.0],
    [V30_NORMAL_ENTRY_MIN, 0.14, 0.17, V30_NORMAL_ENTRY_MAX],
  ))


def v30_progressive_brake_cap(v_ego):
  return float(interp(
    v_ego,
    [8.0, 15.0, 25.0, 35.0],
    [V30_NORMAL_BRAKE_CAP_MIN, 0.27, 0.29, V30_NORMAL_BRAKE_CAP_MAX],
  ))


def v25l_powertrain_decel_cap(v_ego):
  """Maximum 0x01 desired-speed offset below vEgo for regen/coasting."""
  return float(interp(
    v_ego,
    [0.0, 4.0, 15.0, 25.0, 35.0],
    [0.08, 0.15, 0.28, 0.40, 0.45],
  ))


def v25l_encode_hev_brake(brake_cmd):
  """Return (negative pump reaction, legacy combined raw magnitude)."""
  brake_cmd = float(clip(brake_cmd, V25L_BRAKE_MIN, V25R_URGENT_BRAKE_MAX))
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

    # V2.5O hydraulic state, stop-and-go latch, and continuous desired-speed
    # offset for acceleration, coasting, and regenerative deceleration.
    self.v25l_apply_brake = 0.0
    self.v25l_brake_target = 0.0
    self.v25l_brake_active = False
    self.v25l_brake_entry_counter = 0
    self.v25o_nolead_entry_counter = 0
    self.v25o_brake_mode = V25O_BRAKE_MODE_NONE
    self.v25o_sng_armed = False
    self.v25o_stop_hold = False
    self.v25o_sng_release_frames = 0
    self.v25l_lead_counter = 0
    self.v25l_brake_reentry_frame = 0
    self.v25l_propulsion_block_until_frame = 0
    self.v25l_speed_offset = 0.0
    self.v25v_regen_release_until_frame = 0

    # V2.5R planner-primary handoff state. Subscribers are non-blocking and
    # avoid changing DragonPilot's CarController.update call signature.
    self.v25r_release_counter = 0
    self.v25r_lead_accel_counter = 0
    self.v25r_lead_loss_counter = 0
    self.v25r_urgent_brake = False
    self.v30_handoff_counter = 0
    self.v30_handoff_active = False

    # V3.2R low-speed-only handoff state.
    self.v32r_neutral_until_frame = 0
    self.v32r_low_speed_arm_start_frame = -1000000
    self.v32r_overshoot_block_until_frame = 0

    self.v33r_low_speed_guard_until_frame = 0
    self.v33r_stopped_lead_counter = 0
    self.v33r_hold_resume_counter = 0
    self.v33r_release_freeze_until_frame = 0
    self.v33r_release_lead_until_frame = 0
    self.v33r_target_slope_unlock_frame = 0
    self.v33r_overshoot_counter = 0
    self.v33r_overshoot_block_until_frame = 0
    self.v33r_filtered_aego = 0.0

    self.v25r_plan_source = ""
    self.v25r_plan_accel = 0.0
    self.v25r_plan_accel_next = 0.0
    self.v25r_plan_has_lead = False
    self.v25r_plan_frame = -1000000

    self.v25r_lead0_status = False
    self.v25r_lead0_drel = 0.0
    self.v25r_lead0_vrel = 0.0
    self.v25r_lead1_status = False
    self.v25r_lead1_drel = 0.0
    self.v25r_lead1_vrel = 0.0
    self.v25r_radar_frame = -1000000

    try:
      self.v25r_plan_sm = messaging.SubMaster(["longitudinalPlan"])
    except Exception:
      self.v25r_plan_sm = None

    try:
      self.v25r_radar_sm = messaging.SubMaster(["radarState"])
    except Exception:
      self.v25r_radar_sm = None

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
      self.block_brake_until_frame = frame + 50  # Let engagement settle for 0.5 seconds
      self.v25l_apply_brake = 0.0
      self.v25l_brake_target = 0.0
      self.v25l_brake_active = False
      self.v25l_brake_entry_counter = 0
      self.v25o_nolead_entry_counter = 0
      self.v25o_brake_mode = V25O_BRAKE_MODE_NONE
      self.v25o_sng_armed = False
      self.v25o_stop_hold = False
      self.v25o_sng_release_frames = 0
      self.v25l_lead_counter = 0
      self.v25l_brake_reentry_frame = frame
      self.v25l_propulsion_block_until_frame = frame
      self.v25l_speed_offset = 0.0
      self.v25v_regen_release_until_frame = frame
      self.v25r_release_counter = 0
      self.v25r_lead_accel_counter = 0
      self.v25r_lead_loss_counter = 0
      self.v25r_urgent_brake = False
      self.v30_handoff_counter = 0
      self.v30_handoff_active = False
      self.v32r_neutral_until_frame = frame
      self.v32r_low_speed_arm_start_frame = -1000000
      self.v32r_overshoot_block_until_frame = frame
      self.v33r_low_speed_guard_until_frame = (
        frame + V33R_LOW_SPEED_ENGAGEMENT_GUARD_FRAMES
      )
      self.v33r_stopped_lead_counter = 0
      self.v33r_hold_resume_counter = 0
      self.v33r_release_freeze_until_frame = frame
      self.v33r_release_lead_until_frame = frame
      self.v33r_target_slope_unlock_frame = frame
      self.v33r_overshoot_counter = 0
      self.v33r_overshoot_block_until_frame = frame
      self.v33r_filtered_aego = float(CS.out.aEgo)

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

      t_lookup = 0.35 + 0.07 * CS.out.vEgo

      # Read longitudinalPlan. It is the primary longitudinal direction
      # request; the downstream PID may only add a small same-direction amount.
      if self.v25r_plan_sm is not None:
        try:
          self.v25r_plan_sm.update(0)
          if self.v25r_plan_sm.updated["longitudinalPlan"]:
            long_plan = self.v25r_plan_sm["longitudinalPlan"]
            self.v25r_plan_source = v25r_normalize_plan_source(
              getattr(long_plan, "longitudinalPlanSource", "")
            )
            self.v25r_plan_has_lead = bool(
              getattr(long_plan, "hasLead", False)
            )
            plan_accels = getattr(long_plan, "accels", [])
            if len(plan_accels) > 0:
              self.v25r_plan_accel = float(plan_accels[0])
              self.v25r_plan_accel_next = (
                float(plan_accels[1]) if len(plan_accels) > 1
                else self.v25r_plan_accel
              )
            self.v25r_plan_frame = frame
        except Exception:
          pass

      # Radar values are used only to classify urgency. Planner source remains
      # the authority for whether a lead is longitudinally relevant.
      if self.v25r_radar_sm is not None:
        try:
          self.v25r_radar_sm.update(0)
          if self.v25r_radar_sm.updated["radarState"]:
            radar_state = self.v25r_radar_sm["radarState"]
            lead0 = radar_state.leadOne
            lead1 = radar_state.leadTwo
            self.v25r_lead0_status = bool(lead0.status)
            self.v25r_lead0_drel = float(lead0.dRel)
            self.v25r_lead0_vrel = float(lead0.vRel)
            self.v25r_lead1_status = bool(lead1.status)
            self.v25r_lead1_drel = float(lead1.dRel)
            self.v25r_lead1_vrel = float(lead1.vRel)
            self.v25r_radar_frame = frame
        except Exception:
          pass

      plan_fresh = (
        frame - self.v25r_plan_frame <= V25R_PLAN_MAX_AGE_FRAMES
      )
      radar_fresh = (
        frame - self.v25r_radar_frame <= V25R_RADAR_MAX_AGE_FRAMES
      )
      planner_source_lead = (
        plan_fresh and self.v25r_plan_source in ("lead0", "lead1")
      )
      # V2.5T: longitudinalPlanSource identifies which trajectory currently
      # limits the MPC, not whether a lead exists. The MPC can legitimately
      # report source="cruise" while hasLead remains true. Treat hasLead as the
      # lead-presence authority and retain source only for selecting lead0/lead1.
      planner_reports_lead = (
        plan_fresh and
        (self.v25r_plan_has_lead or planner_source_lead)
      )

      # Blend current and next planner acceleration only for anticipation.
      planner_accel_request = (
        0.70 * self.v25r_plan_accel +
        0.30 * self.v25r_plan_accel_next
        if plan_fresh else apply_accel
      )
      planner_brake_request = max(0.0, -planner_accel_request)

      if (
        self.v25r_plan_source == "lead1" and
        self.v25r_lead1_status
      ):
        selected_lead_status = True
        selected_lead_drel = self.v25r_lead1_drel
        selected_lead_vrel = self.v25r_lead1_vrel
      else:
        # Camera-only logs can select lead1 while radarState.leadOne remains
        # the only populated estimate. Use leadOne for urgency in that case.
        selected_lead_status = self.v25r_lead0_status
        selected_lead_drel = self.v25r_lead0_drel
        selected_lead_vrel = self.v25r_lead0_vrel

      if not radar_fresh:
        selected_lead_status = bool(lead)
        selected_lead_drel = 0.0
        selected_lead_vrel = 0.0

      closing_speed = max(0.0, -selected_lead_vrel)
      ttc = (
        selected_lead_drel / closing_speed
        if selected_lead_status and closing_speed > 0.1
        else 999.0
      )

      selected_lead_speed = max(
        0.0,
        CS.out.vEgo + selected_lead_vrel,
      )

      # Qualify camera leads before allowing hydraulic 0x21.
      if lead:
        self.v25l_lead_counter = min(
          self.v25l_lead_counter + 1,
          V25L_LEAD_TRUST_FRAMES,
        )
      else:
        self.v25l_lead_counter = 0

      trusted_lead = self.v25l_lead_counter >= V25L_LEAD_TRUST_FRAMES
      urgent_lead = self.v25l_lead_counter >= V25L_URGENT_LEAD_FRAMES

      if lead:
        self.v25r_lead_loss_counter = 0
      else:
        self.v25r_lead_loss_counter = min(
          self.v25r_lead_loss_counter + 1,
          V25R_LEAD_LOSS_FRAMES,
        )

      # V2.5T: hasLead establishes lead relevance. longitudinalPlanSource can
      # flicker between lead0/lead1/cruise while the same physical lead remains
      # valid, so using source as a gate can suppress hydraulic braking and SNG.
      relevant_lead = (
        trusted_lead and planner_reports_lead
        if plan_fresh else trusted_lead
      )
      relevant_urgent_lead = (
        urgent_lead and planner_reports_lead
        if plan_fresh else urgent_lead
      )
      nolead_planner_context = (
        not planner_reports_lead if plan_fresh else not trusted_lead
      )
      curve_planner_context = (
        plan_fresh and
        not planner_reports_lead and
        self.v25r_plan_source == "turn"
      )

      distance_val = int(clip(
        getattr(CS, "op_distance_val", 1),
        0,
        2,
      ))
      lead_entry_planner, lead_normal_cap, lead_accel_cap = (
        v25r_distance_profile(distance_val)
      )
      lead_hydraulic_entry = lead_entry_planner
      if CS.out.vEgo >= V30_HIGHWAY_MIN_SPEED:
        lead_hydraulic_entry = max(
          lead_entry_planner,
          v30_progressive_hydraulic_entry(CS.out.vEgo),
        )

      # The same setting is now delivered to the MPC by
      # carState.distanceLines; no Params write is needed here.

      urgent_closing = (
        relevant_urgent_lead and
        (
          planner_brake_request >= V25R_URGENT_PLANNER_DECEL or
          (
            closing_speed >= V25R_URGENT_CLOSING_SPEED and
            ttc <= V25R_URGENT_TTC
          )
        )
      )

      stopped_lead_approach = (
        relevant_lead and
        selected_lead_status and
        CS.out.vEgo <= V25O_SNG_ARM_SPEED and
        0.0 < selected_lead_drel <= V25U_STOP_LEAD_MAX_DISTANCE and
        selected_lead_speed <= V25U_STOP_LEAD_MAX_SPEED and
        closing_speed >= V25U_STOP_LEAD_MIN_CLOSING
      )

      # V2.5X-SC1: after the normal logic has already armed stop-and-go, keep a
      # narrowly scoped final-crawl qualifier that does not require a minimum
      # closing speed. The old closing-speed condition naturally becomes false
      # near zero and could let braking taper before physical standstill.
      stop_completion_active = (
        self.v25o_sng_armed and
        self.v25o_brake_mode == V25O_BRAKE_MODE_LEAD and
        relevant_lead and
        selected_lead_status and
        not CS.out.standstill and
        CS.out.vEgo <= V25X_SC_MAX_EGO_SPEED and
        0.0 < selected_lead_drel <= V25X_SC_MAX_LEAD_DISTANCE and
        selected_lead_speed <= V25X_SC_MAX_LEAD_SPEED
      )

      stopped_lead_candidate = (
        selected_lead_status and
        bool(lead) and
        (planner_reports_lead if plan_fresh else True) and
        V33R_STOP_HOLD_MIN_DISTANCE <= selected_lead_drel <=
          V33R_STOP_HOLD_MAX_DISTANCE and
        selected_lead_speed <= V33R_STOPPED_LEAD_MAX_SPEED
      )
      if stopped_lead_candidate:
        self.v33r_stopped_lead_counter = min(
          self.v33r_stopped_lead_counter + 1,
          V33R_STOP_LEAD_TRUST_FRAMES,
        )
      else:
        self.v33r_stopped_lead_counter = 0

      v33r_trusted_stopped_lead = (
        self.v33r_stopped_lead_counter >=
        V33R_STOP_LEAD_TRUST_FRAMES
      )
      v33r_creep_stop_guard = (
        v33r_trusted_stopped_lead and
        not CS.out.standstill and
        V25L_MIN_MOVING_SPEED < CS.out.vEgo <=
          V33R_CREEP_GUARD_MAX_EGO and
        closing_speed >= V33R_CREEP_GUARD_MIN_CLOSING
      )
      v33r_early_highway_entry = (
        relevant_lead and
        selected_lead_status and
        CS.out.vEgo >= V33R_EARLY_HIGHWAY_MIN_SPEED and
        closing_speed >= V33R_EARLY_HIGHWAY_CLOSING and
        ttc <= V33R_EARLY_HIGHWAY_TTC and
        selected_lead_drel <= max(
          35.0,
          CS.out.vEgo * V33R_EARLY_HIGHWAY_TIME_GAP,
        ) and
        planner_brake_request >= V33R_EARLY_HIGHWAY_PLANNER_BRAKE
      )

      self.v33r_filtered_aego += V33R_AEGO_FILTER_ALPHA * (
        float(CS.out.aEgo) - self.v33r_filtered_aego
      )
      v33r_critical_closing = (
        selected_lead_status and
        closing_speed >= V33R_DECEL_GOVERNOR_CRITICAL_CLOSING and
        ttc <= V33R_DECEL_GOVERNOR_CRITICAL_TTC
      )

      # -----------------------------
      # ACC_BRAKE / 0x271 state map from logs
      # -----------------------------
      # 0x00 + pump 0.0 + mag 200 = disabled / neutral / no brake
      # 0x01 + pump 0.0 + mag 200 = enabled / ready / no brake
      # 0x21 + pump -0.4 + mag 0x04xx = active braking request
      # 0x31 = final stop approach; 0x30 = standstill hold (verified in rlogs)

      if not enabled:  # OP is not engaged
        brake_state = 0x00  # Use disabled/neutral brake state
        pump_reaction = 0.0  # No pump reaction
        brake_mag = 200  # Stock neutral magnitude 0x00C8

      else:  # OP is engaged but not necessarily braking
        brake_state = 0x01  # Use enabled/no-brake state found in logs
        pump_reaction = 0.0  # No pump reaction while not braking
        brake_mag = 200  # Stock neutral magnitude 0x00C8

      # -----------------------------
      # V2.5R planner-primary hydraulic braking and stop-and-go
      # -----------------------------
      brake_request = apply_brake

      control_allowed = (
        enabled and
        CS.out.cruiseState.enabled and
        not pcm_cancel_cmd and
        not CS.out.gasPressed and
        not CS.out.brakePressed
      )
      moving_allowed = control_allowed and not CS.out.standstill

      def start_v33r_staged_release():
        self.v33r_release_freeze_until_frame = max(
          self.v33r_release_freeze_until_frame,
          frame + V33R_RELEASE_FREEZE_FRAMES,
        )
        self.v33r_release_lead_until_frame = max(
          self.v33r_release_lead_until_frame,
          frame + V33R_RELEASE_LEAD_HOLD_FRAMES,
        )
        self.v33r_target_slope_unlock_frame = max(
          self.v33r_target_slope_unlock_frame,
          frame + V33R_TARGET_SLOPE_UNLOCK_FRAMES,
        )
        self.v25l_propulsion_block_until_frame = max(
          self.v25l_propulsion_block_until_frame,
          frame + V33R_RELEASE_PROPULSION_BLOCK_FRAMES,
        )
        self.v25l_speed_offset = min(0.0, self.v25l_speed_offset)

      def clear_hydraulic(reentry=True, propulsion_dwell=True):
        self.v25l_apply_brake = 0.0
        self.v25l_brake_target = 0.0
        self.v25l_brake_active = False
        self.v25l_brake_entry_counter = 0
        self.v25o_nolead_entry_counter = 0
        self.v25o_brake_mode = V25O_BRAKE_MODE_NONE
        self.v25r_release_counter = 0
        self.v25r_urgent_brake = False
        self.v30_handoff_counter = 0
        self.v30_handoff_active = False
        if reentry:
          self.v25l_brake_reentry_frame = frame + V25L_REENTRY_BLOCK_FRAMES
        if propulsion_dwell:
          self.v25l_propulsion_block_until_frame = (
            frame + V25L_PROPULSION_DWELL_FRAMES
          )

      # Release 0x30 only when the lead itself moves and the planner remains
      # positive. A transient downstream PID output cannot release the hold.
      if self.v25o_stop_hold:
        hold_safety_exit = not control_allowed
        hold_resume_candidate = (
          selected_lead_status and
          bool(lead) and
          (planner_reports_lead if plan_fresh else True) and
          selected_lead_speed >= V33R_HOLD_RESUME_LEAD_SPEED and
          (
            planner_accel_request >= V25O_SNG_RELEASE_ACCEL
            if plan_fresh else
            apply_accel >= V25O_SNG_RELEASE_ACCEL
          )
        )
        if hold_resume_candidate:
          self.v33r_hold_resume_counter = min(
            self.v33r_hold_resume_counter + 1,
            V33R_HOLD_RESUME_FRAMES,
          )
        else:
          self.v33r_hold_resume_counter = 0

        hold_resume = (
          self.v33r_hold_resume_counter >= V33R_HOLD_RESUME_FRAMES
        )

        if hold_safety_exit or hold_resume:
          self.v25o_stop_hold = False
          self.v33r_hold_resume_counter = 0
          clear_hydraulic(reentry=False, propulsion_dwell=False)
          if hold_safety_exit:
            self.v25o_sng_armed = False
            self.v25o_sng_release_frames = 0
            self.v33r_release_lead_until_frame = frame
            self.v33r_release_freeze_until_frame = frame
          else:
            self.v25o_sng_release_frames = V25O_SNG_RELEASE_FRAMES
            start_v33r_staged_release()
        else:
          self.v25l_brake_active = True
          self.v25o_brake_mode = V25O_BRAKE_MODE_LEAD
          self.v25l_apply_brake = V25O_SNG_HOLD_BRAKE
          self.v25l_brake_target = V25O_SNG_HOLD_BRAKE
          self.v25l_speed_offset = 0.0

      sng_release_active = False
      if self.v25o_sng_release_frames > 0:
        if not control_allowed:
          self.v25o_sng_release_frames = 0
          self.v25o_sng_armed = False
        else:
          sng_release_active = True
          self.v25o_sng_release_frames -= 1
          self.v25l_brake_active = False
          self.v25o_brake_mode = V25O_BRAKE_MODE_NONE
          self.v25l_apply_brake = 0.0
          self.v25l_brake_target = 0.0
          self.v25l_speed_offset = 0.0

      planner_allows_hold = (
        planner_accel_request < V25O_SNG_RELEASE_ACCEL
        if plan_fresh else
        apply_accel < V25O_SNG_RELEASE_ACCEL
      )
      direct_standstill_hold = (
        control_allowed and
        CS.out.vEgo <= V33R_STOP_HOLD_MAX_EGO and
        v33r_trusted_stopped_lead and
        planner_allows_hold
      )
      approached_standstill_hold = (
        self.v25o_sng_armed and
        CS.out.standstill and
        control_allowed and
        planner_allows_hold
      )
      if (
        not sng_release_active and
        not self.v25o_stop_hold and
        (direct_standstill_hold or approached_standstill_hold)
      ):
        self.v25o_stop_hold = True
        self.v25o_sng_armed = True
        self.v25l_brake_active = True
        self.v25o_brake_mode = V25O_BRAKE_MODE_LEAD
        self.v25l_apply_brake = V25O_SNG_HOLD_BRAKE
        self.v25l_brake_target = V25O_SNG_HOLD_BRAKE
        self.v25l_speed_offset = 0.0
        self.v33r_release_lead_until_frame = max(
          self.v33r_release_lead_until_frame,
          frame + V33R_RELEASE_LEAD_HOLD_FRAMES,
        )

      soft_releasing_hydraulic = False
      hydraulic_handoff_release = False

      if not sng_release_active and not self.v25o_stop_hold and self.v25l_brake_active:
        safety_hard_release = not moving_allowed
        force_soft_release = False

        v30_handoff_candidate = (
          self.v25o_brake_mode == V25O_BRAKE_MODE_LEAD and
          CS.out.vEgo >= V30_HIGHWAY_MIN_SPEED and
          plan_fresh and
          relevant_lead and
          not urgent_closing and
          not self.v25r_urgent_brake and
          not self.v25o_sng_armed and
          not stop_completion_active and
          planner_brake_request < lead_hydraulic_entry and
          apply_accel >= V30_HANDOFF_PID_ACCEL and
          CS.out.aEgo <= V30_HANDOFF_AEGO_MAX
        )

        if (
          v30_handoff_candidate and
          not self.v30_handoff_active
        ):
          self.v30_handoff_counter = min(
            self.v30_handoff_counter + 1,
            V30_HANDOFF_FRAMES,
          )
          if (
            self.v30_handoff_counter >=
            V30_HANDOFF_FRAMES
          ):
            self.v30_handoff_active = True
        elif not self.v30_handoff_active:
          self.v30_handoff_counter = 0

        if self.v30_handoff_active:
          # Complete the hydraulic release once latched. The existing
          # planner-negative desired-speed path will take over as regen after
          # hydraulic_req clears; positive propulsion remains dwell-blocked.
          force_soft_release = True
          hydraulic_handoff_release = True

        if self.v25o_brake_mode == V25O_BRAKE_MODE_LEAD:
          # Raw lead loss is a safety exit. A downstream positive PID request
          # is intentionally ignored while the lead planner still wants decel.
          if self.v25r_lead_loss_counter >= V25R_LEAD_LOSS_FRAMES:
            safety_hard_release = True
          elif plan_fresh and not planner_reports_lead:
            self.v25o_sng_armed = False
            if (
              moving_allowed and
              CS.out.vEgo > V25O_NOLEAD_MIN_SPEED and
              planner_brake_request >= 0.22 and
              brake_request >= V25O_NOLEAD_BRAKE_ENTRY
            ):
              self.v25o_brake_mode = V25O_BRAKE_MODE_NOLEAD
              self.v25l_apply_brake = min(
                self.v25l_apply_brake, V25O_NOLEAD_BRAKE_MAX
              )
              self.v25l_brake_target = min(
                self.v25l_brake_target, V25O_NOLEAD_BRAKE_MAX
              )
              self.v25r_urgent_brake = False
            else:
              force_soft_release = True

        elif self.v25o_brake_mode == V25O_BRAKE_MODE_NOLEAD:
          if CS.out.vEgo <= V25O_NOLEAD_MIN_SPEED:
            safety_hard_release = True
          elif (
            relevant_lead and
            planner_brake_request >= lead_entry_planner
          ):
            self.v25o_brake_mode = V25O_BRAKE_MODE_LEAD
          elif (
            plan_fresh and
            planner_brake_request < V25R_PLANNER_RELEASE
          ):
            force_soft_release = True

        elif self.v25o_brake_mode == V25W_BRAKE_MODE_CURVE:
          if CS.out.vEgo <= V25O_NOLEAD_MIN_SPEED:
            safety_hard_release = True
          elif (
            relevant_lead and
            planner_brake_request >= lead_entry_planner
          ):
            self.v25o_brake_mode = V25O_BRAKE_MODE_LEAD
          elif (
            plan_fresh and
            not curve_planner_context and
            planner_brake_request < 0.12
          ):
            # Allow a brief source flicker while the turn deceleration remains
            # meaningful, then taper normally once the turn request clears.
            force_soft_release = True

        if self.v25o_brake_mode == V25O_BRAKE_MODE_LEAD:
          hold_brake_to_standstill = (
            self.v25o_sng_armed and
            (
              stopped_lead_approach or
              stop_completion_active
            ) and
            CS.out.vEgo <= V25O_SNG_ARM_SPEED
          )
          low_demand = (
            False
            if hold_brake_to_standstill else
            (
              planner_brake_request < V25R_PLANNER_RELEASE
              if plan_fresh else
              brake_request < 0.12
            )
          )
        elif self.v25o_brake_mode == V25W_BRAKE_MODE_CURVE:
          low_demand = (
            planner_brake_request < V25W_CURVE_BRAKE_RELEASE
            if plan_fresh else
            brake_request < V25O_NOLEAD_BRAKE_RELEASE
          )
        else:
          low_demand = (
            planner_brake_request < 0.08
            if plan_fresh else
            brake_request < V25O_NOLEAD_BRAKE_RELEASE
          )

        if safety_hard_release:
          self.v25o_sng_armed = False
          clear_hydraulic()

        elif force_soft_release or low_demand:
          self.v25r_release_counter = min(
            self.v25r_release_counter + 1,
            V25R_RELEASE_CONFIRM_FRAMES,
          )
          soft_releasing_hydraulic = True
          self.v25l_brake_target = V25L_BRAKE_MIN
          release_step = (
            V30_HANDOFF_STEP_DOWN
            if hydraulic_handoff_release else
            V25R_BRAKE_STEP_DOWN
          )
          self.v25l_apply_brake = max(
            0.0,
            self.v25l_apply_brake - release_step,
          )

          if (
            self.v25r_release_counter >= V25R_RELEASE_CONFIRM_FRAMES and
            self.v25l_apply_brake <= V25L_BRAKE_MIN
          ):
            self.v25o_sng_armed = False
            was_v30_handoff = self.v30_handoff_active
            start_v33r_staged_release()
            clear_hydraulic()
            if was_v30_handoff:
              self.v25l_brake_reentry_frame = (
                frame + V30_REENTRY_BLOCK_FRAMES
              )
        else:
          self.v25r_release_counter = 0

      if not sng_release_active and not self.v25o_stop_hold and not self.v25l_brake_active:
        stopped_lead_reentry = (
          (
            stopped_lead_approach and
            planner_brake_request >= V25U_STOP_LEAD_MIN_BRAKE
          ) or
          v33r_creep_stop_guard
        )
        lead_entry = (
          moving_allowed and
          (relevant_lead or v33r_creep_stop_guard) and
          (
            planner_brake_request >= lead_hydraulic_entry or
            stopped_lead_reentry or
            v33r_early_highway_entry
          ) and
          CS.out.vEgo > (
            V25L_MIN_MOVING_SPEED
            if v33r_creep_stop_guard else
            V25L_MIN_ENTRY_SPEED
          ) and
          (
            frame > self.block_brake_until_frame or
            v33r_creep_stop_guard
          ) and
          (
            frame >= self.v25l_brake_reentry_frame or
            stopped_lead_reentry
          )
        )
        urgent_entry = (
          moving_allowed and
          urgent_closing and
          planner_brake_request >= 0.25 and
          CS.out.vEgo > V25L_MIN_ENTRY_SPEED and
          frame > self.block_brake_until_frame
        )
        curve_entry = (
          moving_allowed and
          curve_planner_context and
          planner_brake_request >= V25W_CURVE_BRAKE_ENTRY and
          CS.out.vEgo > V25O_NOLEAD_MIN_SPEED and
          frame > self.block_brake_until_frame and
          frame >= self.v25l_brake_reentry_frame and
          brake_request >= V25W_CURVE_PID_ENTRY
        )
        nolead_entry = (
          moving_allowed and
          nolead_planner_context and
          not curve_planner_context and
          planner_brake_request >= 0.25 and
          CS.out.vEgo > V25O_NOLEAD_MIN_SPEED and
          frame > self.block_brake_until_frame and
          frame >= self.v25l_brake_reentry_frame and
          brake_request >= V25O_NOLEAD_BRAKE_ENTRY
        )

        if lead_entry:
          required_entry_frames = (
            V33R_CREEP_ENTRY_FRAMES
            if v33r_creep_stop_guard else
            V25R_LEAD_ENTRY_FRAMES
          )
          self.v25l_brake_entry_counter = min(
            self.v25l_brake_entry_counter + 1,
            required_entry_frames,
          )
        else:
          required_entry_frames = V25R_LEAD_ENTRY_FRAMES
          self.v25l_brake_entry_counter = 0

        if urgent_entry:
          self.v25o_nolead_entry_counter = min(
            self.v25o_nolead_entry_counter + 1,
            V25R_URGENT_ENTRY_FRAMES,
          )
        elif curve_entry:
          self.v25o_nolead_entry_counter = min(
            self.v25o_nolead_entry_counter + 1,
            V25W_CURVE_ENTRY_FRAMES,
          )
        elif nolead_entry:
          self.v25o_nolead_entry_counter = min(
            self.v25o_nolead_entry_counter + 1,
            V25O_NOLEAD_ENTRY_FRAMES,
          )
        else:
          self.v25o_nolead_entry_counter = 0

        urgent_confirmed = (
          urgent_entry and
          self.v25o_nolead_entry_counter >= V25R_URGENT_ENTRY_FRAMES
        )
        normal_confirmed = (
          self.v25l_brake_entry_counter >= required_entry_frames
        )

        if urgent_confirmed or normal_confirmed:
          self.v25l_brake_active = True
          self.v25o_brake_mode = V25O_BRAKE_MODE_LEAD
          self.v25r_urgent_brake = bool(urgent_confirmed)
          self.v25l_apply_brake = V25L_BRAKE_MIN
          self.v25l_brake_target = V25L_BRAKE_MIN
          self.v25l_brake_entry_counter = 0
          self.v25o_nolead_entry_counter = 0
          self.v25r_release_counter = 0
          self.v25l_speed_offset = 0.0
          if v33r_creep_stop_guard:
            self.v25o_sng_armed = True
            self.v25l_apply_brake = max(
              self.v25l_apply_brake,
              V33R_CREEP_BRAKE_FLOOR,
            )
            self.v25l_brake_target = max(
              self.v25l_brake_target,
              V33R_CREEP_BRAKE_FLOOR,
            )

        elif (
          curve_entry and
          self.v25o_nolead_entry_counter >= V25W_CURVE_ENTRY_FRAMES
        ):
          self.v25l_brake_active = True
          self.v25o_brake_mode = V25W_BRAKE_MODE_CURVE
          self.v25r_urgent_brake = False
          self.v25l_apply_brake = V25L_BRAKE_MIN
          self.v25l_brake_target = V25L_BRAKE_MIN
          self.v25l_brake_entry_counter = 0
          self.v25o_nolead_entry_counter = 0
          self.v25o_sng_armed = False
          self.v25r_release_counter = 0
          self.v25l_speed_offset = 0.0

        elif (
          nolead_entry and
          self.v25o_nolead_entry_counter >= V25O_NOLEAD_ENTRY_FRAMES
        ):
          self.v25l_brake_active = True
          self.v25o_brake_mode = V25O_BRAKE_MODE_NOLEAD
          self.v25r_urgent_brake = False
          self.v25l_apply_brake = V25L_BRAKE_MIN
          self.v25l_brake_target = V25L_BRAKE_MIN
          self.v25l_brake_entry_counter = 0
          self.v25o_nolead_entry_counter = 0
          self.v25o_sng_armed = False
          self.v25r_release_counter = 0
          self.v25l_speed_offset = 0.0

      if self.v25l_brake_active and not self.v25o_stop_hold:
        if not soft_releasing_hydraulic:
          speed_scale = interp(
            CS.out.vEgo,
            [0.0, 140.0 * CV.KPH_TO_MS],
            [1.0, 1.0 / 1.5]
          )

          if self.v25o_brake_mode == V25O_BRAKE_MODE_NOLEAD:
            brake_cap = V25O_NOLEAD_BRAKE_MAX
          elif self.v25o_brake_mode == V25W_BRAKE_MODE_CURVE:
            brake_cap = v25w_curve_brake_cap(CS.out.vEgo)
          else:
            if urgent_closing or planner_brake_request >= V25R_URGENT_HARD_DECEL:
              self.v25r_urgent_brake = True

            if self.v25r_urgent_brake:
              requested_cap = V25R_URGENT_BRAKE_MAX
            elif CS.out.vEgo >= V30_HIGHWAY_MIN_SPEED:
              requested_cap = min(
                lead_normal_cap,
                v30_progressive_brake_cap(CS.out.vEgo),
              )
            else:
              requested_cap = lead_normal_cap

            brake_cap = min(
              requested_cap,
              v25l_low_speed_brake_cap(CS.out.vEgo),
              v25l_high_speed_brake_cap(CS.out.vEgo),
            )
            if stop_completion_active:
              brake_cap = max(brake_cap, V25X_SC_MIN_BRAKE)

          if plan_fresh:
            target_request = planner_brake_request
            # Downstream PID may add only a small same-direction correction.
            if apply_accel < 0.0:
              pid_extra = max(0.0, brake_request - planner_brake_request)
              target_request += min(
                V25R_PID_BRAKE_ALLOWANCE,
                V25R_PID_BRAKE_BLEND * pid_extra,
              )
          else:
            target_request = brake_request

          if self.v25o_brake_mode in (
            V25O_BRAKE_MODE_NOLEAD,
            V25W_BRAKE_MODE_CURVE,
          ):
            target_request = min(
              target_request,
              planner_brake_request + V25L_BRAKE_MIN,
            )

          raw_target_brake = float(clip(
            target_request * speed_scale,
            V25L_BRAKE_MIN,
            brake_cap,
          ))
          if v33r_creep_stop_guard:
            raw_target_brake = max(
              raw_target_brake,
              V33R_CREEP_BRAKE_FLOOR,
            )
          if (
            self.v33r_filtered_aego <= V33R_DECEL_GOVERNOR_START and
            not v33r_critical_closing
          ):
            raw_target_brake = min(
              raw_target_brake,
              max(
                V25L_BRAKE_MIN,
                self.v25l_apply_brake -
                V33R_DECEL_GOVERNOR_STEP_DOWN,
              ),
            )
          if stop_completion_active:
            raw_target_brake = max(
              raw_target_brake,
              V25X_SC_MIN_BRAKE,
            )
          filter_alpha = (
            V25R_BRAKE_FILTER_UP
            if raw_target_brake > self.v25l_brake_target
            else V25R_BRAKE_FILTER_DOWN
          )
          self.v25l_brake_target += filter_alpha * (
            raw_target_brake - self.v25l_brake_target
          )
          self.v25l_brake_target = float(clip(
            self.v25l_brake_target,
            V25L_BRAKE_MIN,
            brake_cap,
          ))
          if self.v25l_brake_target > self.v25l_apply_brake:
            v30_step_up = (
              V30_BRAKE_STEP_UP_URGENT
              if self.v25r_urgent_brake else
              V30_BRAKE_STEP_UP
            )
            rate_limited_brake = min(
              self.v25l_brake_target,
              self.v25l_apply_brake + v30_step_up,
            )
          else:
            rate_limited_brake = max(
              self.v25l_brake_target,
              self.v25l_apply_brake - V25R_BRAKE_STEP_DOWN,
            )
          self.v25l_apply_brake = min(brake_cap, rate_limited_brake)

        if (
          self.v25o_brake_mode == V25O_BRAKE_MODE_LEAD and
          relevant_lead and
          CS.out.vEgo <= V25O_SNG_ARM_SPEED and
          (
            planner_brake_request >= V25O_SNG_ARM_BRAKE or
            (
              stopped_lead_approach and
              planner_brake_request >= V25U_STOP_LEAD_MIN_BRAKE
            ) or
            v33r_creep_stop_guard
          )
        ):
          self.v25o_sng_armed = True

        self.v25l_propulsion_block_until_frame = max(
          self.v25l_propulsion_block_until_frame,
          frame + V25L_PROPULSION_DWELL_FRAMES,
        )

      if (
        self.v25o_sng_armed and
        not self.v25o_stop_hold and
        self.v25o_sng_release_frames == 0 and
        CS.out.vEgo > V25O_SNG_APPROACH_SPEED and
        planner_accel_request >= V25R_PLANNER_ACCEL_ENTRY
      ):
        self.v25o_sng_armed = False

      hydraulic_req = (
        self.v25l_brake_active and
        self.v25l_apply_brake >= V25L_BRAKE_MIN
      )

      if hydraulic_req:
        if self.v25o_stop_hold:
          brake_state = 0x30
          self.v25l_apply_brake = V25O_SNG_HOLD_BRAKE
        elif (
          self.v25o_brake_mode == V25O_BRAKE_MODE_LEAD and
          self.v25o_sng_armed and
          CS.out.vEgo <= V25O_SNG_APPROACH_SPEED
        ):
          brake_state = 0x31
        else:
          brake_state = 0x21

        pump_reaction, brake_mag = v25l_encode_hev_brake(
          self.v25l_apply_brake
        )

      elif sng_release_active:
        # Verified stock release/staging combination: state 0x01, pump
        # -0.4/+0.4, and neutral magnitude byte C8.
        brake_state = 0x01
        pump_reaction = -0.4
        brake_mag = (4 << 8) | 200

      # -----------------------------
      # V2.5R planner-primary desired-speed arbitration
      # -----------------------------
      # The planner selects direction. The downstream PID cannot request gas
      # while the lead planner still requests slowing. This is the key fix for
      # brake -> rev/no acceleration -> jump -> brake oscillation.
      release_freeze_active = (
        enabled and frame < self.v33r_release_freeze_until_frame
      )
      release_lead_active = (
        enabled and frame < self.v33r_release_lead_until_frame
      )
      low_speed_engagement_guard = (
        enabled and
        CS.out.vEgo < V33R_LOW_SPEED_ENGAGEMENT_MAX and
        frame < self.v33r_low_speed_guard_until_frame
      )

      target_slope_lock = (
        hydraulic_req or
        planner_brake_request >= V33R_TARGET_SLOPE_BRAKE or
        release_freeze_active or
        (
          self.v25l_speed_offset < -V25V_REGEN_OFFSET_EPS and
          CS.out.aEgo <= V33R_TARGET_SLOPE_AEGO
        )
      )
      if target_slope_lock:
        self.v33r_target_slope_unlock_frame = max(
          self.v33r_target_slope_unlock_frame,
          frame + V33R_TARGET_SLOPE_UNLOCK_FRAMES,
        )

      propulsion_blocked = (
        hydraulic_req or
        frame < self.v25l_propulsion_block_until_frame or
        frame < self.v33r_target_slope_unlock_frame or
        frame < self.v33r_overshoot_block_until_frame or
        release_freeze_active or
        low_speed_engagement_guard
      )

      if plan_fresh:
        if planner_accel_request < 0.0:
          # Deceleration planner wins even if PID temporarily reverses positive.
          effective_accel = planner_accel_request
        elif planner_accel_request > 0.0 and apply_accel > 0.0:
          # Positive propulsion requires planner/PID agreement.
          effective_accel = min(
            planner_accel_request + 0.08,
            0.75 * planner_accel_request + 0.25 * apply_accel,
          )
        else:
          # Planner positive but PID still negative: coast rather than fight.
          effective_accel = 0.0
      else:
        effective_accel = apply_accel

      # Preserve V3.0's normal 0.30 s regen-release dwell at every speed.
      # Below 30.6 km/h, add an isolated 0.50 s neutral stage before the
      # low-speed ECU lead-bit wake can begin. This never changes highway
      # desired-speed shaping.
      regen_or_brake_active = (
        hydraulic_req or
        sng_release_active or
        effective_accel <= -V25L_DECEL_DEADBAND or
        self.v25l_speed_offset < -V25V_REGEN_OFFSET_EPS
      )
      if regen_or_brake_active:
        self.v25v_regen_release_until_frame = max(
          self.v25v_regen_release_until_frame,
          frame + V25V_REGEN_RELEASE_DWELL_FRAMES,
        )
        if CS.out.vEgo < V32R_LOW_SPEED_MAX:
          self.v32r_neutral_until_frame = max(
            self.v32r_neutral_until_frame,
            frame + V32R_REGEN_NEUTRAL_DWELL_FRAMES,
          )
          self.v32r_low_speed_arm_start_frame = -1000000

      if planner_reports_lead and plan_fresh:
        if (
          planner_accel_request >= V25R_PLANNER_ACCEL_ENTRY and
          apply_accel > 0.0
        ):
          self.v25r_lead_accel_counter = min(
            self.v25r_lead_accel_counter + 1,
            V25R_LEAD_ACCEL_CONFIRM_FRAMES,
          )
        else:
          self.v25r_lead_accel_counter = 0
      else:
        self.v25r_lead_accel_counter = V25R_LEAD_ACCEL_CONFIRM_FRAMES

      lead_accel_confirmed = (
        self.v25r_lead_accel_counter >= V25R_LEAD_ACCEL_CONFIRM_FRAMES
      )

      # V3.2R low-speed lead classification is used only to decide whether the
      # outgoing ECU wake bit may be used. It does not alter planner/radar lead
      # state and cannot enter hydraulic braking.
      positive_agreement = (
        (
          planner_accel_request >= V25L_ACCEL_ENTRY and
          apply_accel > 0.0
        )
        if plan_fresh else
        apply_accel >= V25L_ACCEL_ENTRY
      )
      departing_lead = (
        selected_lead_status and
        selected_lead_vrel >= V32R_DEPARTING_LEAD_VREL and
        selected_lead_drel >= V32R_DEPARTING_LEAD_DREL
      )
      distant_nonclosing_lead = (
        selected_lead_status and
        selected_lead_drel >= V32R_NONBLOCKING_LEAD_DISTANCE and
        selected_lead_vrel >= V32R_NONBLOCKING_LEAD_VREL
      )
      lead_nonblocking_for_propulsion = (
        not selected_lead_status or
        departing_lead or
        distant_nonclosing_lead
      )
      low_speed_propulsion_request = (
        control_allowed and
        not hydraulic_req and
        not sng_release_active and
        not self.v25o_stop_hold and
        CS.out.vEgo < V32R_LOW_SPEED_MAX and
        positive_agreement and
        lead_nonblocking_for_propulsion
      )

      low_speed_arm_eligible = (
        low_speed_propulsion_request and
        frame >= self.v32r_neutral_until_frame and
        frame >= self.v32r_overshoot_block_until_frame and
        self.v25l_speed_offset >= -V25V_REGEN_OFFSET_EPS
      )
      if not low_speed_arm_eligible:
        self.v32r_low_speed_arm_start_frame = -1000000
      elif self.v32r_low_speed_arm_start_frame < 0:
        self.v32r_low_speed_arm_start_frame = frame

      low_speed_arm_active = (
        low_speed_arm_eligible and
        frame - self.v32r_low_speed_arm_start_frame <
        V32R_LOW_SPEED_ARM_FRAMES
      )
      low_speed_arm_complete = (
        low_speed_arm_eligible and
        not low_speed_arm_active
      )

      unexpected_positive_accel = (
        control_allowed and
        CS.out.aEgo >= V33R_OVERSHOOT_AEGO and
        (
          self.v25l_speed_offset <= V25V_REGEN_OFFSET_EPS or
          planner_accel_request <= V25R_PLANNER_ACCEL_ENTRY
        )
      )
      if unexpected_positive_accel:
        self.v33r_overshoot_counter = min(
          self.v33r_overshoot_counter + 1,
          V33R_OVERSHOOT_CONFIRM_FRAMES,
        )
      else:
        self.v33r_overshoot_counter = 0

      if (
        self.v33r_overshoot_counter >=
        V33R_OVERSHOOT_CONFIRM_FRAMES
      ):
        self.v25l_speed_offset = 0.0
        self.v33r_overshoot_block_until_frame = (
          frame + V33R_OVERSHOOT_BLOCK_FRAMES
        )
        self.v33r_target_slope_unlock_frame = max(
          self.v33r_target_slope_unlock_frame,
          frame + V33R_TARGET_SLOPE_UNLOCK_FRAMES,
        )
        self.v33r_release_lead_until_frame = max(
          self.v33r_release_lead_until_frame,
          frame + V33R_RELEASE_LEAD_HOLD_FRAMES,
        )
        self.v33r_overshoot_counter = 0

        # Never add no-lead braking. A persistent close stopped lead may
        # re-enter the verified 0x31/0x30 stop path.
        if (
          v33r_trusted_stopped_lead and
          CS.out.vEgo <= V33R_CREEP_GUARD_MAX_EGO
        ):
          self.v25o_sng_armed = True
          self.v25l_brake_active = True
          self.v25o_brake_mode = V25O_BRAKE_MODE_LEAD
          self.v25l_apply_brake = max(
            self.v25l_apply_brake,
            V33R_CREEP_BRAKE_FLOOR,
          )
          self.v25l_brake_target = max(
            self.v25l_brake_target,
            V33R_CREEP_BRAKE_FLOOR,
          )
          hydraulic_req = True

      low_speed_overshoot = (
        CS.out.vEgo < V32R_LOW_SPEED_MAX and
        self.v25l_speed_offset > 0.0 and
        CS.out.aEgo >= V32R_LOW_SPEED_OVERSHOOT_AEGO
      )
      if low_speed_overshoot:
        self.v25l_speed_offset = 0.0
        self.v32r_overshoot_block_until_frame = (
          frame + V32R_OVERSHOOT_BLOCK_FRAMES
        )
        self.v32r_neutral_until_frame = max(
          self.v32r_neutral_until_frame,
          frame + V32R_REGEN_NEUTRAL_DWELL_FRAMES,
        )
        self.v32r_low_speed_arm_start_frame = -1000000

      if not control_allowed:
        self.v25l_speed_offset = 0.0
        self.v33r_release_freeze_until_frame = frame
        self.v33r_release_lead_until_frame = frame
        self.v33r_target_slope_unlock_frame = frame
        self.v33r_overshoot_counter = 0

      elif hydraulic_req or sng_release_active:
        self.v25l_speed_offset = 0.0

      elif (
        effective_accel <= -V25L_DECEL_DEADBAND and
        frame > self.block_brake_until_frame and
        frame >= self.v25l_brake_reentry_frame
      ):
        effective_brake = max(0.0, -effective_accel)
        target_offset = -min(
          v25l_powertrain_decel_cap(CS.out.vEgo),
          effective_brake * t_lookup,
        )

        if self.v25l_speed_offset > 0.0:
          self.v25l_speed_offset = 0.0
        elif target_offset < self.v25l_speed_offset:
          self.v25l_speed_offset = max(
            target_offset,
            self.v25l_speed_offset - V25L_DECEL_OFFSET_STEP_DOWN,
          )
        else:
          if (
            frame >= self.v33r_target_slope_unlock_frame and
            CS.out.aEgo >= V33R_PROPULSION_AEGO_MIN
          ):
            self.v25l_speed_offset = min(
              target_offset,
              self.v25l_speed_offset + V33R_TARGET_RETURN_STEP,
            )

      elif (
        effective_accel >= V25L_ACCEL_ENTRY and
        not propulsion_blocked and
        frame >= self.v25v_regen_release_until_frame and
        CS.out.aEgo >= V33R_PROPULSION_AEGO_MIN and
        (
          not planner_reports_lead or
          not plan_fresh or
          lead_accel_confirmed or
          low_speed_propulsion_request
        ) and
        (
          CS.out.vEgo >= V32R_LOW_SPEED_MAX or
          low_speed_arm_complete
        )
      ):
        # At and above 30.6 km/h this is the exact V3.0 cap/ramp selection.
        # Only the new low-speed path receives the smaller V3.2R authority.
        if CS.out.vEgo < V32R_LOW_SPEED_MAX:
          if departing_lead:
            accel_cap = max(
              lead_accel_cap,
              V32R_DEPARTING_ACCEL_CAP,
            )
            offset_cap = V32R_DEPARTING_OFFSET_CAP
            accel_step_up = V32R_DEPARTING_OFFSET_STEP_UP
          else:
            accel_cap = V32R_LOW_SPEED_ACCEL_CAP
            offset_cap = V32R_LOW_SPEED_OFFSET_CAP
            accel_step_up = V32R_LOW_SPEED_OFFSET_STEP_UP
        else:
          accel_cap = (
            lead_accel_cap
            if planner_reports_lead and plan_fresh else
            V25L_ACCEL_CAP
          )
          offset_cap = (
            0.30
            if planner_reports_lead and plan_fresh else
            V25L_ACCEL_OFFSET_MAX
          )
          accel_step_up = V25L_ACCEL_OFFSET_STEP_UP

        target_offset = min(
          offset_cap,
          min(effective_accel, accel_cap) * t_lookup,
        )

        if self.v25l_speed_offset < 0.0:
          self.v25l_speed_offset = min(
            0.0,
            self.v25l_speed_offset + V33R_TARGET_RETURN_STEP,
          )
        elif target_offset > self.v25l_speed_offset:
          self.v25l_speed_offset = min(
            target_offset,
            self.v25l_speed_offset + accel_step_up,
          )
        else:
          self.v25l_speed_offset = max(
            target_offset,
            self.v25l_speed_offset - V25L_ACCEL_OFFSET_STEP_DOWN,
          )

      else:
        # Planner neutral, unconfirmed acceleration, or regen-release dwell:
        # return to a true zero offset before any positive propulsion.
        if self.v25l_speed_offset < 0.0:
          if (
            frame >= self.v33r_target_slope_unlock_frame and
            CS.out.aEgo >= V33R_PROPULSION_AEGO_MIN
          ):
            self.v25l_speed_offset = min(
              0.0,
              self.v25l_speed_offset + V33R_TARGET_RETURN_STEP,
            )
        else:
          self.v25l_speed_offset = 0.0

      if release_freeze_active:
        self.v25l_speed_offset = min(0.0, self.v25l_speed_offset)

      if low_speed_arm_active:
        # ECU wake stage: lead bit only, no positive desired-speed target.
        self.v25l_speed_offset = 0.0

      if not enabled:
        des_speed = 0.0
      elif self.v25o_stop_hold or sng_release_active:
        # Stock stop and pump-release logs keep ACC_CMD at zero.
        des_speed = 0.0
      elif hydraulic_req:
        # Never combine lowered 0x273 desired speed with hydraulic 0x271.
        des_speed = CS.out.vEgo
      else:
        des_speed = max(0.0, CS.out.vEgo + self.v25l_speed_offset)

      # Select the two independent 0x273 longitudinal mode bits explicitly.
      # Stock-observed state table:
      #   disabled                 = 0x00: IS_ACCEL=0, IS_DECEL=0
      #   normal 0x01, including
      #   lowered ACC_CMD regen    = 0x40: IS_ACCEL=1, IS_DECEL=0
      #   moving 0x21 / crawl 0x31 = 0x20: IS_ACCEL=0, IS_DECEL=1
      #   standstill 0x30          = 0x60: IS_ACCEL=1, IS_DECEL=1
      #   0x30 pressure release    = 0x20 during the FC/04/C8 staging frames
      if not enabled:
        acc_cmd_is_accel = False
        acc_cmd_is_decel = False
      elif brake_state == 0x30:
        acc_cmd_is_accel = True
        acc_cmd_is_decel = True
      elif brake_state in (0x21, 0x31) or sng_release_active:
        acc_cmd_is_accel = False
        acc_cmd_is_decel = True
      else:
        # Neutral 0x01 uses the normal 0x40 state even when ACC_CMD is slightly
        # below vEgo for smooth engine/regen deceleration.
        acc_cmd_is_accel = True
        acc_cmd_is_decel = False

      low_speed_accel_unlock = (
        enabled and
        low_speed_propulsion_request and
        not hydraulic_req and
        not sng_release_active and
        not self.v25o_stop_hold and
        frame >= self.v32r_neutral_until_frame and
        frame >= self.v32r_overshoot_block_until_frame and
        (
          low_speed_arm_active or
          self.v25l_speed_offset > 0.0
        )
      )
      # Cancel/disabled frames cannot preserve a stale lead bit.
      lead_for_acc_cmd = bool(
        enabled and
        (
          lead or
          low_speed_accel_unlock or
          release_lead_active or
          self.v25o_stop_hold
        )
      )

      can_sends.append(  # Add ACC_CMD_HUD message
        dnga_create_accel_command(  # Build 0x273 ACC command frame
          self.packer,  # CAN packer
          CS.cruise_speed,  # OP cruise set speed
          CS.out.cruiseState.available,  # ACC ready/available bit
          enabled,  # OP enabled state
          lead_for_acc_cmd,  # Real lead or isolated low-speed ECU wake
          des_speed,  # Desired speed command
          acc_cmd_is_accel,  # Explicit stock IS_ACCEL mode bit
          acc_cmd_is_decel,  # Explicit stock IS_DECEL mode bit
          CS.op_distance_val  # Follow distance setting
        )
      )

      can_sends.append(  # Add ACC_BRAKE message
        dnga_create_brake_command(  # Build 0x271 brake command
          self.packer,  # CAN packer
          brake_state,  # 0x00, 0x01, 0x21, 0x31, or 0x30
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
