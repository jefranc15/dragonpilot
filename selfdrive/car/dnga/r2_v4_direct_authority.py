"""R2 V4 direct moving-brake authority: OFFLINE DEVELOPMENT, NOT ROAD VALIDATED.

Unlike the earlier experimental combiner, this moving-brake state machine does
not use V5.0's coast/READY/hydraulic decision or magnitude as a normal floor.
It uses R2's independent planner proposal. V5.0 remains a low-speed/SNG and
explicit urgent-brake fallback; no Panda policy is modified here.

Command-field correlations from stock ACC are not calibrated deceleration,
pressure, motor torque, or regenerated battery current.
"""
from dataclasses import dataclass, replace
from math import isfinite

from selfdrive.car.dnga.r2_v4_shadow_controller import BRAKING, READY

HOLD = 0x30
CREEPING = 0x31
DISABLED = 0x00
R2_MIN_ENTRY_SPEED_MPS = 5.6
ENTRY_STRENGTH = 0.08
NORMAL_STEP = 0.02
URGENT_STEP = 0.05
RELEASE_STEP = 0.04
MAX_STRENGTH = 0.87


def strength_of(magnitude):
  return max(0.0, min(MAX_STRENGTH, (200 - (int(magnitude) & 0xff)) / 100.0))


def encode(strength):
  strength = max(ENTRY_STRENGTH, min(MAX_STRENGTH, float(strength)))
  pump = 5 if strength >= 0.75 else 4
  return (pump << 8) | int(round(200 - 100 * strength)), -pump / 10.0


@dataclass(frozen=True)
class R2DirectStatus:
  authority: str
  reason: str
  strength: float
  expected_275: int
  feedback_fault: str


class R2V4DirectAuthority:
  """Twenty-Hz command producer for a separate bench-only integration.

  Its normal moving BRAKING and READY decisions come solely from the V4
  proposal, never from the V5.0 coast-first handoff. V5.0 still provides
  low-speed HOLD/CREEPING and urgent fallback. Caller must keep the final
  positive-acceleration interlock after this stage.
  """

  def __init__(self):
    self.reset()

  def reset(self):
    self.last_strength = 0.0
    self.owned_braking = False
    self.in_release = False

  def _status(self, authority, reason, strength=0.0, fault=""):
    expected = 0
    if authority == "r2_braking":
      magnitude, _ = encode(strength)
      expected = int(round(-20.48 * (200 - (magnitude & 0xff))))
    return R2DirectStatus(authority, reason, strength, expected, fault)

  def apply(self, base, proposal, *, enabled, speed_mps, override=False,
            urgent=False, engagement_guard=False, feedback_fresh=False,
            friction_active=False):
    # Inactive driver override and unavailable planner remain under the
    # original controller. Never synthesize a brake on invalid inputs.
    if (not enabled or override or not base.enabled or
        base.brake_state == DISABLED):
      self.reset()
      return base, self._status("base", "inactive")

    if not isfinite(float(speed_mps)) or speed_mps < 0:
      self.reset()
      return base, self._status("base", "invalid_speed")

    if proposal is None or proposal.brake_state not in (BRAKING, READY):
      self.reset()
      return base, self._status("base", "invalid_or_stale_plan")

    # Maintain established low-speed/SNG behavior. At a speed-boundary
    # handoff, only yield while the base controller already has a physical
    # braking/hold command. Never silently drop an ongoing R2 brake to READY.
    if base.brake_state in (HOLD, CREEPING):
      self.reset()
      return base, self._status("base", "sng")

    if (speed_mps < R2_MIN_ENTRY_SPEED_MPS and not self.owned_braking
        and not self.in_release):
      self.reset()
      return base, self._status("base", "below_entry_speed")

    if (speed_mps < R2_MIN_ENTRY_SPEED_MPS and self.owned_braking
        and base.brake_state == BRAKING):
      self.reset()
      return base, self._status("base", "moving_to_low_speed_braking")

    if engagement_guard and not self.owned_braking:
      return base, self._status("base", "engagement_guard")

    fault = ""
    if not feedback_fresh:
      fault = "stale_brake_feedback"
    if getattr(proposal, "missing_275_response", False):
      fault = "no_expected_275_response"

    if proposal.brake_state == BRAKING:
      target = max(ENTRY_STRENGTH, min(MAX_STRENGTH, strength_of(proposal.magnitude)))
      # Safety exception: an urgent V5.0 request is not weakened. Ordinary
      # base brake demand does NOT control the direct R2 decision.
      emergency_floor = (strength_of(base.magnitude)
                         if urgent and base.brake_state == BRAKING else 0.0)
      if not self.owned_braking:
        strength = max(ENTRY_STRENGTH, emergency_floor)
      else:
        step = URGENT_STEP if urgent else NORMAL_STEP
        strength = max(emergency_floor,
                       min(target, self.last_strength + step))
        if strength < self.last_strength:
          strength = max(strength, self.last_strength - RELEASE_STEP)
      strength = max(ENTRY_STRENGTH, min(MAX_STRENGTH, strength))
      magnitude, pump = encode(strength)
      self.last_strength = strength
      self.owned_braking = True
      self.in_release = False
      output = replace(base, brake_state=BRAKING, pump=pump,
                       magnitude=magnitude,
                       speed=min(speed_mps, max(0.0, proposal.target_kph / 3.6)),
                       is_accel=False, is_decel=True)
      return output, self._status("r2_braking", "direct_1216_entry_or_ramp",
                                  strength, fault)

    self.last_strength = 0.0
    # If V5.0 detected an urgent physical-braking need, don't discard it just
    # because R2 issued READY. This is an emergency-only fallback.
    if urgent and base.brake_state == BRAKING:
      self.owned_braking = False
      self.in_release = False
      return base, self._status("base", "urgent_brake_fallback", fault=fault)

    if self.owned_braking:
      self.owned_braking = False
      self.in_release = True

    if (self.in_release and proposal.magnitude == 1224 and
        proposal.pump == -0.4):
      output = replace(base, brake_state=READY, magnitude=1224, pump=-0.4,
                       speed=min(speed_mps, max(0.0, proposal.target_kph / 3.6)),
                       is_accel=False, is_decel=True)
      return output, self._status("r2_release", "stock_style_ready", fault=fault)

    self.in_release = False
    # Genuine no-brake demand is READY (not an artificial 1216 request).
    # Positive propulsion still uses the V5.0 speed shaping, but is gated
    # by the V4 planner/feedback clearance plus the final interlock.
    if proposal.target_gap_kph > 0.05:
      output = replace(base, brake_state=READY, magnitude=200, pump=0.0,
                       speed=min(speed_mps, max(0.0, proposal.target_kph / 3.6)),
                       is_accel=False, is_decel=True)
      return output, self._status("r2_ready", "target_gap_return", fault=fault)

    accel_ok = bool(proposal.is_accel and feedback_fresh and
                    not friction_active and not fault and base.is_accel)
    output = replace(base, brake_state=READY, magnitude=200, pump=0.0,
                     speed=(base.speed if accel_ok else speed_mps),
                     is_accel=accel_ok, is_decel=False)
    return output, self._status("r2_ready", "feedback_gated_accel_or_coast",
                                fault=fault)
