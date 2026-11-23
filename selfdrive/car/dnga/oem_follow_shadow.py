"""V7.1 DNGA OEM-follow *shadow* arbitration. NO ACTUATION.

Observe OEM camera 0x271/0x273 as an inseparable longitudinal intention
and compare against openpilot's *vision-turn* longitudinalPlan. Factory
radar/lead identity stays OEM-owned. The computed candidate curve target and
magnitude/pump suggestions MUST NEVER be sent to CAN.

Why shadow: the Nov 17 V6 highway logs include a +1.89 m/s² no-pedal
post-brake launch. A smaller 0x271 magnitude paired with untouched 0x273
can cause underbraking or hybrid engine-braking compensation. OP-generated
curve braking against stock IS_ACCEL=1 can recreate the ECU state mismatch.
"""
from dataclasses import dataclass
import math

# Frame counts at 100 Hz controller updates, shadow evaluated every ACC_STEP.
CAMERA_MAX_AGE_FRAMES = 15
CAMERA_MAX_SKEW_FRAMES = 5
PLAN_MAX_AGE_FRAMES = 50
RELEASE_MONITOR_FRAMES = 80
CURVE_CONFIRM_SAMPLES = 3
CURVE_SPEED_MARGIN_MPS = 0.70
SUSPECTED_LAUNCH_AEGO = 0.80
# Used only to flag *hypothetical* weakening by the V7 brake shadow.
WEAKENING_WARN_FRACTION = 0.20

BRAKING_STATES = frozenset((0x21, 0x30, 0x31))
IDLE_STATES = frozenset((0x00, 0x01))


@dataclass(frozen=True)
class CameraSnapshot:
  brake_rx_frame: int
  acc_rx_frame: int
  brake_state: int
  brake_magnitude: int
  pump: float
  enabled: bool
  lead: bool
  is_accel: bool
  is_decel: bool
  desired_speed_kph: float
  set_speed_kph: float


@dataclass(frozen=True)
class TurnSnapshot:
  rx_frame: int = -1000000
  source: str = ""
  has_lead: bool = False
  turn_speed_mps: float = 0.0
  near_accel: float = 0.0
  lookahead_accel: float = 0.0


@dataclass(frozen=True)
class OEMFollowObservation:
  frame: int
  mode: str
  reason: str
  camera_lead: bool
  camera_braking: bool
  camera_acceleration: bool
  camera_deceleration: bool
  factory_acc_kph: float
  factory_set_kph: float
  hypothetical_curve_target_kph: float
  camera_brake_strength: float
  hypothetical_brake_strength: float
  camera_pump: float
  hypothetical_pump: float
  weakening_fraction: float
  # Descriptive information only: no raw CAN, arbitration command or
  # actuator modification is produced by this module.
  transmittable: bool = False


def normalize_source(source):
  s = str(source).strip().lower()
  return s.rsplit(".", 1)[-1]


def finite(value, fallback=0.0):
  try:
    x = float(value)
    return x if math.isfinite(x) else fallback
  except (TypeError, ValueError):
    return fallback


class OEMFollowShadow:
  def __init__(self):
    self.last_frame = -1
    self.previous_camera_braking = False
    self.historical_previous_op_braking = False
    self.release_until_frame = -1
    self.turn_confirm_count = 0
    self.last_observation = None

  def observe(self, frame, camera, turn, *,
              ego_speed_mps, ego_accel_mps2,
              gas_pressed=False, brake_pressed=False,
              brake_shadow=None, historical_op_brake_state=None):
    """Evaluate one synchronized snapshot, never return a CAN command.

    brake_shadow is V7's read-only StockBrakeObservation for the same frame.
    historical_op_brake_state is an OPTIONAL decoded 0x271 from an older
    V6 OP-TX rlog; it must never come from the V7/V7.1 live controller.
    Both factory-camera and historical OP release events are observed.
    No camera data is used as physical ECU acknowledgment.
    """
    if frame <= self.last_frame:
      raise ValueError("shadow samples must have strictly increasing frames")
    self.last_frame = frame
    ego_speed = max(0.0, finite(ego_speed_mps))
    ego_accel = finite(ego_accel_mps2)
    factory_target = finite(camera.desired_speed_kph)
    factory_set = finite(camera.set_speed_kph)
    camera_braking = camera.brake_state in BRAKING_STATES
    camera_age_brake = frame - int(camera.brake_rx_frame)
    camera_age_acc = frame - int(camera.acc_rx_frame)
    valid_camera = (
      0 <= camera_age_brake <= CAMERA_MAX_AGE_FRAMES
      and 0 <= camera_age_acc <= CAMERA_MAX_AGE_FRAMES
      and abs(camera.brake_rx_frame - camera.acc_rx_frame) <= CAMERA_MAX_SKEW_FRAMES
      and camera.brake_state in (BRAKING_STATES | IDLE_STATES)
      and 0 <= camera.brake_magnitude <= 200
      and math.isfinite(finite(camera.pump, float("nan")))
      and factory_target >= 0.0 and factory_set >= 0.0
    )
    if self.previous_camera_braking and not camera_braking and valid_camera:
      self.release_until_frame = frame + RELEASE_MONITOR_FRAMES
    self.previous_camera_braking = camera_braking if valid_camera else False

    # November 17 V6 event: OP's bus-0 command entered READY while the
    # Toyota camera stayed READY throughout. The camera-only release
    # detector never sees this. Separate offline V6-TX replay observation
    # therefore tracks the actual OP BRAKING->READY edge independently.
    if historical_op_brake_state is not None:
      op_state = int(historical_op_brake_state)
      if self.historical_previous_op_braking and op_state in IDLE_STATES:
        self.release_until_frame = frame + RELEASE_MONITOR_FRAMES
      self.historical_previous_op_braking = op_state in BRAKING_STATES

    camera_strength = ((200 - camera.brake_magnitude) / 100.0
                       if camera_braking and valid_camera else 0.0)
    # The existing V7 shadow may substantially weaken OEM first braking.
    # Quantify the *hypothetical* reduced magnitude and proportional pump
    # for research ONLY; never feed these suggestions to an output message.
    shadow_strength = (
      max(0.0, finite(brake_shadow.proposed_strength))
      if brake_shadow is not None and brake_shadow.valid and valid_camera
      else camera_strength
    )
    pump = max(0.0, finite(camera.pump))
    ratio = min(1.0, shadow_strength / camera_strength) if camera_strength > 0.0 else 1.0
    shadow_pump = pump * ratio if camera_braking else pump
    weakening = (1.0 - ratio) if camera_strength > 0.0 else 0.0

    turn_age = frame - int(turn.rx_frame)
    plan_fresh = 0 <= turn_age <= PLAN_MAX_AGE_FRAMES
    source = normalize_source(turn.source)
    confirmed_source = plan_fresh and source == "turn" and not turn.has_lead
    turn_speed = finite(turn.turn_speed_mps)
    meaningful_curve = (confirmed_source and
                        turn_speed > 0.0 and
                        ego_speed > turn_speed + CURVE_SPEED_MARGIN_MPS and
                        min(finite(turn.near_accel), finite(turn.lookahead_accel)) < -0.05)
    self.turn_confirm_count = min(CURVE_CONFIRM_SAMPLES, self.turn_confirm_count + 1) if meaningful_curve else 0

    mode = "OEM_CRUISE"
    reason = "factory_camera_owns_speed_and_braking"
    curve_target = factory_target

    # Priority: invalid, overrides, factory lead/braking, release, then turn.
    if not valid_camera:
      mode, reason = "INVALID_CAMERA_PAIR", "camera_271_273_stale_skewed_or_invalid"
      self.turn_confirm_count = 0
    elif gas_pressed or brake_pressed:
      mode, reason = "DRIVER_OVERRIDE", "driver_pedal_active"
      self.turn_confirm_count = 0
    elif not camera.enabled:
      mode, reason = "OEM_INACTIVE", "camera_acc_not_enabled_or_aeb_separate"
      self.turn_confirm_count = 0
    elif camera.lead:
      mode, reason = "OEM_LEAD", "factory_lead_has_priority_over_vision_turn"
      self.turn_confirm_count = 0
    elif camera_braking or (camera.is_decel and (not meaningful_curve or camera.is_accel)):
      mode, reason = "OEM_DECEL", "factory_braking_or_decel_has_priority"
      self.turn_confirm_count = 0
    elif frame <= self.release_until_frame:
      if ego_accel >= SUSPECTED_LAUNCH_AEGO and not camera.is_decel:
        mode, reason = "POST_BRAKE_SURGE_WARNING", "positive_ego_accel_during_release_monitor"
      else:
        mode, reason = "RELEASE_MONITOR", "observe_hybrid_recovery_no_override"
      self.turn_confirm_count = 0
    elif plan_fresh and turn.has_lead and not camera.lead:
      # Never promote OP lead detections into OEM lead control authority.
      mode, reason = "LEAD_SOURCE_DISAGREEMENT", "OP_plan_lead_not_OEM_lead"
      self.turn_confirm_count = 0
    elif meaningful_curve:
      if camera.is_accel:
        mode, reason = "CURVE_PROPULSION_CONFLICT", "OEM_is_accel_on_while_OP_turn_decel"
      elif not camera.is_decel:
        mode, reason = "CURVE_UNAUTHORIZED", "OEM_no_lead_no_decel_standalone_braking_unsafe"
      else:
        mode, reason = "CURVE_SHADOW_CANDIDATE", "OEM_decel_present_but_curve_requires_ECU_validation"
      if self.turn_confirm_count >= CURVE_CONFIRM_SAMPLES:
        curve_target = max(0.0, min(factory_target, turn_speed * 3.6))
    elif source == "turn" and not plan_fresh:
      mode, reason = "STALE_TURN_PLAN", "vision_turn_not_fresh"
    elif not plan_fresh:
      mode, reason = "NO_FRESH_PLAN", "OEM_cruise_only"

    if camera_braking and weakening > WEAKENING_WARN_FRACTION:
      # Do not replace factory lead/braking priority; attach safety warning.
      reason += "|shadow_proposal_weakens_OEM_braking"

    result = OEMFollowObservation(
      frame, mode, reason,
      bool(camera.lead), bool(camera_braking),
      bool(camera.is_accel), bool(camera.is_decel),
      factory_target, factory_set, curve_target,
      camera_strength, shadow_strength,
      pump, shadow_pump, weakening,
    )
    self.last_observation = result
    return result
