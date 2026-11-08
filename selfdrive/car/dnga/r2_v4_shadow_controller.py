"""R2 V4 stock-coordinated longitudinal candidate. SHADOW / DIAGNOSTIC ONLY.

This module cannot generate, pack, or transmit CAN. It must never be used as the
road-going longitudinal authority without independent ECU-response validation.
Its empirical target relationship fits *factory command fields*, not physical
regen power, hydraulic pressure, motor torque, or a guaranteed braking rate.
"""
from dataclasses import dataclass
from math import isfinite

READY = 0x01
BRAKING = 0x21
DISABLED = 0x00
TRUSTED_SOURCES = ("turn", "lead0", "lead1")


@dataclass(frozen=True)
class V4Observation:
  brake_state: int
  pump: float
  magnitude: int
  target_kph: float
  is_decel: bool
  is_accel: bool
  requested_brake: float
  target_gap_kph: float
  source_bridge: bool
  early_preview: bool
  outside_stock_calibration: bool
  missing_275_response: bool
  feedback_clear_count: int
  release_frames_left: int
  reason: str


class R2V4ShadowController:
  """20 Hz observer, stateful between actual received frames. No ECU control."""
  def __init__(self):
    self.reset()

  def reset(self):
    self.gap_kph = 0.0
    self.last_frame = None
    self.last_braking = False
    self.last_trusted = False
    self.release_frames = 0
    self.clear_frames = 0
    self.last_feedback_seq = None
    self.no_response_frames = 0

  def step(self, *, frame, v_ego_mps, planned_accel, preview_accel,
           downstream_pid, measured_accel, source, has_lead, enabled,
           driver_override=False, feedback_275=None, friction_08c=None,
           feedback_fresh=False, feedback_sequence=None,
           positive_accel_permitted=False, urgent=False):
    vals = (v_ego_mps, planned_accel, preview_accel, downstream_pid, measured_accel)
    if not all(isfinite(float(v)) for v in vals) or v_ego_mps < 0:
      raise ValueError("invalid R2V4 observation inputs")
    frame = int(frame)
    source = str(source).lower().split(".")[-1]
    v_kph = float(v_ego_mps) * 3.6
    contiguous = (self.last_frame is not None and 0 < frame - self.last_frame <= 15)
    direct = bool(enabled and not driver_override and (has_lead or source in TRUSTED_SOURCES))
    agreement = planned_accel <= -0.14 and downstream_pid <= -0.05
    not_overbraking = measured_accel >= planned_accel - 0.18
    bridge = bool(enabled and not driver_override and not direct and
                  source == "cruise" and contiguous and self.last_braking and
                  self.last_trusted and agreement and not_overbraking)
    authorized = direct or bridge

    rx_fresh = bool(feedback_fresh and feedback_275 is not None and friction_08c is not None)
    new_rx = bool(rx_fresh and feedback_sequence is not None and
                  feedback_sequence != self.last_feedback_seq)
    if new_rx:
      self.last_feedback_seq = feedback_sequence

    if not enabled or driver_override:
      self.reset()
      return V4Observation(DISABLED, 0.0, 200, v_kph, False, False,
                           0.0, 0.0, False, False, False, False,
                           0, 0, "inactive")

    future_brake = max(0.0, -float(preview_accel)) * 0.55
    immediate_brake = max(0.0, -float(planned_accel))
    demand = max(immediate_brake, future_brake) if authorized else 0.0
    braking = demand > 0.01
    early = braking and immediate_brake <= 0.01
    outside = bool(braking and not (7.70522 <= v_kph <= 76.97496
                                    and 0.08 <= demand <= 1.46))

    if braking:
      encoded = min(0.87, max(0.08, demand))
      pump_byte = 5 if encoded >= 0.75 else 4
      low = max(0, min(255, round(200 - 100 * encoded)))
      magnitude = (pump_byte << 8) | low
      pump = -pump_byte / 10.0
      gap_target = min(v_kph, 10.7, max(
        0.05, -0.23432350383566514 + 6.72517749 * encoded + 0.03370613 * v_kph
      ))
      self.release_frames = 0
      self.clear_frames = 0
      if new_rx and int(feedback_275) >= 0:
        self.no_response_frames += 1
      elif new_rx:
        self.no_response_frames = 0
    else:
      magnitude, pump = 200, 0.0
      gap_target = 0.0
      self.no_response_frames = 0
      if self.last_braking:
        self.release_frames = 24
        self.clear_frames = 0
      if new_rx and int(feedback_275) == 0 and int(friction_08c) == 0:
        self.clear_frames = min(3, self.clear_frames + 1)
      else:
        self.clear_frames = 0
      if self.release_frames > 0:
        magnitude, pump = 1224, -0.4
        self.release_frames -= 1

    # Smooth the *gap*, not the raw speed field; urgent control unvalidated.
    max_up = 0.60 if urgent else 0.18
    step = max(-0.35, min(max_up, gap_target - self.gap_kph))
    self.gap_kph = max(0.0, min(v_kph, self.gap_kph + step))

    surge_veto = bool(downstream_pid < -0.05 or
                      (measured_accel > 0.25 and downstream_pid < 0.05))
    accel = bool(not braking and rx_fresh and new_rx and
                 self.clear_frames >= 3 and self.gap_kph <= 0.05 and
                 positive_accel_permitted and planned_accel > 0 and not surge_veto)
    decel = bool(braking or
                 (not accel and (self.gap_kph > 0.05 or
                  (rx_fresh and (feedback_275 < 0 or friction_08c > 0)))))
    self.last_frame = frame
    self.last_braking = braking
    self.last_trusted = braking and authorized and (direct or bridge)

    reason = ("planner" if direct and braking else
              "source_bridge" if bridge and braking else
              "ready_release" if not braking and magnitude == 1224 else
              "ready")
    if not rx_fresh:
      reason += ":stale_feedback"
    if outside:
      reason += ":outside_stock_domain"
    if self.no_response_frames >= 3:
      reason += ":missing_275"
    return V4Observation(BRAKING if braking else READY, pump, magnitude,
                         max(0.0, v_kph - self.gap_kph), decel, accel,
                         demand, self.gap_kph, bridge, early, outside,
                         self.no_response_frames >= 3, self.clear_frames,
                         self.release_frames, reason)
