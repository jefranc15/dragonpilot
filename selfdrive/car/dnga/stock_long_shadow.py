"""Read-only Toyota DNGA HEV camera-brake filter prototype.

This module NEVER constructs or sends CAN. Its proposed magnitude is
diagnostic only and must never be used as a command without separate bench/HIL
validation against real HEV brake/regen/ABS behavior.
"""

from dataclasses import dataclass


ACTIVE_BRAKE_STATES = frozenset((0x21, 0x30, 0x31))
NEUTRAL_BRAKE_STATES = frozenset((0x00, 0x01))
MAX_CAMERA_AGE_FRAMES = 15
PROPOSED_STEP_UP = 0.025  # hypothetical 50ms ordinary-braking ramp
PROPOSED_PASSTHROUGH_AT = 0.50  # do not even propose smoothing strong braking


@dataclass(frozen=True)
class StockBrakeObservation:
  frame: int
  valid: bool
  stock_state: int
  raw_strength: float
  proposed_strength: float
  raw_magnitude: int
  proposed_magnitude: int
  reason: str


class StockBrakeShadow:
  def __init__(self):
    self.last_frame = -1
    self.last_proposed = 0.0
    self.last_observation = None

  def observe(self, frame, rx_frame, stock_state, magnitude):
    """Return *only* a hypothetical comfort suggestion; never a CAN payload."""
    age = int(frame) - int(rx_frame)
    state = int(stock_state)
    mag = int(magnitude)

    valid = bool(
      0 <= age <= MAX_CAMERA_AGE_FRAMES
      and 0 <= mag <= 200
      and state in (ACTIVE_BRAKE_STATES | NEUTRAL_BRAKE_STATES)
      and frame > self.last_frame
    )
    self.last_frame = frame
    if not valid:
      self.last_proposed = 0.0
      result = StockBrakeObservation(frame, False, state, 0.0, 0.0, mag, mag, "stale_or_invalid_camera")
    else:
      raw = (200 - mag) / 100.0 if state in ACTIVE_BRAKE_STATES else 0.0
      if state not in ACTIVE_BRAKE_STATES:
        proposal = 0.0
        reason = "stock_release_unmodified"
      elif raw >= PROPOSED_PASSTHROUGH_AT:
        proposal = raw
        reason = "strong_brake_unmodified"
      else:
        # Shadow calculation only. A real limiter might weaken necessary
        # braking and must NOT be transmitted without controlled validation.
        proposal = min(raw, self.last_proposed + PROPOSED_STEP_UP)
        reason = "ordinary_brake_shadow_slew"
      self.last_proposed = proposal
      result = StockBrakeObservation(
        frame, True, state, raw, proposal, mag,
        max(0, min(200, int(round(200 - 100 * proposal)))), reason,
      )
    self.last_observation = result
    return result
