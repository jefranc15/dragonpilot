"""R2 V4 experimental longitudinal CAN command combiner.

This module transforms an already calculated V5.0 LongitudinalCommand into an
R2 V4 longitudinal candidate when OpenPilot has an engaged session.

NOT A VERIFIED BRAKING SYSTEM. The 0x273 fit describes factory *command*
values, not guaranteed wheel deceleration or available regenerative torque.
Never treat missing 0x275 as proof of no braking, or negative 0x275 as proof
of negative physical motor torque.
"""
from dataclasses import dataclass, replace
from math import isfinite

from selfdrive.car.dnga.r2_v4_shadow_controller import BRAKING, READY

# V4 acts only on moving BRAKING/READY; stock low-speed, CREEPING and HOLD
# remain under the already existing controller.
CREEPING = 0x31
HOLD = 0x30
DISABLED = 0x00
MIN_V4_SPEED_MPS = 5.6
INITIAL_STRENGTH = 0.08
NORMAL_BRAKE_STEP_UP = 0.02
URGENT_BRAKE_STEP_UP = 0.05
BRAKE_STEP_DOWN = 0.04
MAX_BRAKE_STRENGTH = 0.87


@dataclass(frozen=True)
class R2V4Actuation:
  active: bool
  state_changed: bool
  request_strength: float
  expected_275_raw: int
  reason: str


def magnitude_strength(magnitude):
  """DNGA signed 0x271 brake strength proxy, not hydraulic pressure."""
  return max(0.0, min(MAX_BRAKE_STRENGTH, (200 - (int(magnitude) & 0xff)) * 0.01))


def encode_magnitude(strength):
  """Generate already documented DNGA 0x271 pump/magnitude representation."""
  strength = max(INITIAL_STRENGTH, min(MAX_BRAKE_STRENGTH, float(strength)))
  pump_byte = 5 if strength >= 0.75 else 4
  lower = max(0, min(255, int(round(200 - 100 * strength))))
  return ((pump_byte << 8) | lower, -pump_byte / 10.0)


class R2V4ExperimentalActuator:
  """Moving-only coordinated 0x271 + 0x273 command candidate."""

  def __init__(self):
    self.last_brake_strength = 0.0
    self.last_state = None
    self.last_v4_active = False

  def reset(self):
    self.last_brake_strength = 0.0
    self.last_state = None
    self.last_v4_active = False

  def apply(self, command, proposal, *, enabled, speed_mps, override=False,
            urgent=False, engagement_guard=False):
    """Return (LongitudinalCommand, R2V4Actuation).

    command: existing interlocked V5.0 command; proposal: V4 shadow observation.
    The combiner never changes Panda safety or transmits frames by itself.
    """
    prior = self.last_state
    if (not enabled or override or not command.enabled or
        command.brake_state == DISABLED):
      self.reset()
      return command, R2V4Actuation(False, False, 0.0, 0, "inactive")

    if (proposal is None or not isfinite(float(speed_mps)) or
        (speed_mps < MIN_V4_SPEED_MPS and not self.last_v4_active) or engagement_guard or
        command.brake_state in (HOLD, CREEPING)):
      self.last_brake_strength = 0.0
      self.last_v4_active = False
      self.last_state = command.brake_state
      return command, R2V4Actuation(
        False, prior != command.brake_state,
        magnitude_strength(command.magnitude) if command.brake_state == BRAKING else 0.0,
        0, "existing_controller")

    if proposal.brake_state == BRAKING:
      proposed_strength = magnitude_strength(proposal.magnitude)
      base_strength = (magnitude_strength(command.magnitude)
                       if command.brake_state == BRAKING else 0.0)
      target_strength = max(INITIAL_STRENGTH, proposed_strength, base_strength)
      step_up = URGENT_BRAKE_STEP_UP if urgent else NORMAL_BRAKE_STEP_UP
      # Preserve any existing V5.0 urgent braking rather than weakening it.
      applied_strength = max(
        base_strength,
        min(target_strength, max(INITIAL_STRENGTH,
                                 self.last_brake_strength + step_up)),
      )
      if self.last_brake_strength > applied_strength:
        applied_strength = max(
          applied_strength, self.last_brake_strength - BRAKE_STEP_DOWN
        )
      magnitude, pump = encode_magnitude(applied_strength)
      target_speed = min(float(speed_mps), max(0.0, proposal.target_kph / 3.6))
      output = replace(command,
                       brake_state=BRAKING, magnitude=magnitude, pump=pump,
                       speed=target_speed, is_accel=False, is_decel=True)
      self.last_brake_strength = applied_strength
      self.last_v4_active = True
      self.last_state = BRAKING
      # Reconstructed 0x275 request is only a diagnostic prediction.
      expected = int(round(-20.48 * (200 - (magnitude & 0xff))))
      return output, R2V4Actuation(
        True, prior != BRAKING, applied_strength, expected, "v4_braking")

    self.last_brake_strength = 0.0

    # Do not override already-active V5.0 moving braking with a weaker
    # READY proposal. Existing braking remains authoritative.
    if command.brake_state == BRAKING:
      self.last_state = BRAKING
      return command, R2V4Actuation(
        False, prior != BRAKING, magnitude_strength(command.magnitude), 0,
        "v5_braking_kept")

    if (proposal.brake_state == READY and proposal.magnitude == 1224 and\n        proposal.pump == -0.4 and self.last_v4_active):
      output = replace(command,
                       brake_state=READY, magnitude=1224, pump=-0.4,
                       speed=min(float(speed_mps), max(0.0, proposal.target_kph / 3.6)),
                       is_accel=False, is_decel=True)
      self.last_state = READY
      return output, R2V4Actuation(True, prior != READY, 0.0, 0, "v4_release")

    self.last_v4_active = False
    self.last_state = READY
    if proposal.target_gap_kph > 0.05:
      # Wind the target up progressively; do not accelerate while below ego.
      output = replace(command, brake_state=READY, magnitude=200, pump=0.0,
                       speed=min(float(speed_mps), max(0.0, proposal.target_kph / 3.6)),
                       is_accel=False, is_decel=True)
      return output, R2V4Actuation(True, prior != READY, 0.0, 0, "v4_target_return")

    # Final V5.0 propulsion permission remains required; V4's newer
    # feedback/deceleration veto may additionally block it.
    if not proposal.is_accel and command.is_accel:
      output = replace(command, is_accel=False, is_decel=False,
                       speed=float(speed_mps))
      return output, R2V4Actuation(True, prior != READY, 0.0, 0, "v4_accel_veto")
    return command, R2V4Actuation(True, prior != READY, 0.0, 0, "v4_ready")
