"""R2 planner-derived moving brake decision, integrated in DNGA longitudinal.py.
The fitted stock CAN relationship is not a validated ECU or motor torque model."""
from dataclasses import dataclass
from math import isfinite

READY = 0x01
BRAKING = 0x21
DISABLED = 0x00
TRUSTED_SOURCES = ("turn", "lead0", "lead1")


@dataclass(frozen=True)
class R2Proposal:
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


class R2BrakePolicy:
  """Planner-driven 20 Hz state machine for moving brake requests."""
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
      return R2Proposal(DISABLED, 0.0, 200, v_kph, False, False,
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
    return R2Proposal(BRAKING if braking else READY, pump, magnitude,
                         max(0.0, v_kph - self.gap_kph), decel, accel,
                         demand, self.gap_kph, bridge, early, outside,
                         self.no_response_frames >= 3, self.clear_frames,
                         self.release_frames, reason)


"""R2 1216-first moving brake command encoding. Not vehicle-validated."""
from dataclasses import dataclass, replace
from math import isfinite


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


class R2BrakeActuator:
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


# DNGA v6.0 longitudinal: one active command producer, R2 for moving,
# isolated V5.0 compatibility path for low-speed HOLD/CREEPING only.
# Direct R2 CAN selection remains disabled until verified HIL/ECU response.
#
# Existing V5.0 moving-brake coast/hand-off logic does not enter R2's normal
# moving BRAKING/READY path, and the controller no longer runs a shadow pass.
from cereal import messaging
from selfdrive.car.dnga.dnga_hybrid_feedback import hybrid_brake_feedback_state, hybrid_feedback_snapshot
from selfdrive.car.dnga.longitudinal_v50_legacy import LongitudinalCommand
from selfdrive.car.dnga.longitudinal_kinematic_guard import LongitudinalController as V50LowSpeedController
from selfdrive.car.dnga.values import BrakeState, CarControllerParams



class LongitudinalController:
  """Single R2 command producer selected by CarController for bench integration.

  R2 owns 0x271 moving BRAKING/READY and coordinated 0x273 speed targets.
  An isolated legacy low-speed adapter is preserved for HOLD/CREEPING while
  reworking stop-and-go. Never use this as a public-road brake controller
  without hardware-in-the-loop and control-safety verification.
  """
  def __init__(self):
    self.policy = R2BrakePolicy()
    self.actuator = R2BrakeActuator()
    self.low_speed = V50LowSpeedController()
    self.plan_sm = None
    try:
      self.plan_sm = messaging.SubMaster(["longitudinalPlan"])
    except Exception:
      pass
    self.plan_frame = -1000000
    self.plan_source = ""
    self.plan_accel = 0.0
    self.preview_accel = 0.0
    self.plan_has_lead = False
    self.prev_enabled = False
    self.last_authority = ""
    self.last_feedback_fault = ""

  def _read_plan(self, frame):
    if self.plan_sm is None:
      return
    try:
      self.plan_sm.update(0)
      if not self.plan_sm.updated["longitudinalPlan"]:
        return
      msg = self.plan_sm["longitudinalPlan"]
      self.plan_source = str(getattr(msg, "longitudinalPlanSource", "")).lower().split(".")[-1]
      self.plan_has_lead = bool(getattr(msg, "hasLead", False))
      accels = getattr(msg, "accels", [])
      if accels:
        self.plan_accel = 0.7 * float(accels[0]) + 0.3 * float(
          accels[1] if len(accels) > 1 else accels[0])
        self.preview_accel = min(float(v) for v in accels[:min(13, len(accels))])
      else:
        self.plan_accel = 0.0
        self.preview_accel = 0.0
      self.plan_frame = int(frame)
    except Exception:
      # A stale plan must never authorize a new R2 braking decision.
      self.plan_frame = -1000000

  def update(self, enabled, CS, frame, accel, pcm_cancel_cmd, lead):
    if frame % CarControllerParams.ACC_STEP:
      return None

    session = bool(enabled and CS.out.cruiseState.enabled and
                   not pcm_cancel_cmd and not CS.out.brakePressed and
                   not CS.out.gasPressed)
    v_ego = max(0.0, float(CS.out.vEgo))
    self._read_plan(frame)

    if not session:
      self.policy.reset()
      self.actuator.reset()
      self.prev_enabled = False
      self.last_authority = "inactive"
      self.last_feedback_fault = ""
      return LongitudinalCommand(
        bool(enabled and CS.out.cruiseState.enabled), bool(lead), v_ego,
        False, False, BrakeState.READY, 0.0, 200,
      )

    feedback = hybrid_feedback_snapshot(CS, frame)
    feedback_fresh, physical_braking = hybrid_brake_feedback_state(feedback)
    plan_fresh = bool(0 <= frame - self.plan_frame <= 25)

    # Historical SNG controller is isolated to the low-speed domain. There is
    # no concurrent / shadow V5.0 moving-brake command inside the R2 path.
    # A moving-to-SNG handoff is not yet HIL-validated.
    if v_ego < 5.6 and not self.actuator.owned_braking and not self.actuator.in_release:
      self.policy.reset()
      self.actuator.reset()
      self.last_authority = "legacy_sng"
      self.last_feedback_fault = ""
      return self.low_speed.update(enabled, CS, frame, accel, pcm_cancel_cmd, lead)

    base_accel_agreement = bool(plan_fresh and self.plan_accel > 0.05 and
                                float(accel) > 0.05 and
                                feedback_fresh and not physical_braking)
    base = LongitudinalCommand(
      True, bool(lead or self.plan_has_lead),
      v_ego + (min(0.55, max(0.0, self.plan_accel) * 0.35)
               if base_accel_agreement else 0.0),
      base_accel_agreement, False, BrakeState.READY, 0.0, 200,
    )
    if not plan_fresh:
      self.policy.reset()
      self.actuator.reset()
      self.last_authority = "stale_plan"
      self.last_feedback_fault = "stale_plan"
      return LongitudinalCommand(
        True, bool(lead), v_ego, False, False, BrakeState.READY, 0.0, 200,
      )

    rx275 = int(getattr(CS, "hybrid_feedback_rx_frame_275", -1000000))
    rx08c = int(getattr(CS, "hybrid_feedback_rx_frame_08c", -1000000))
    request = self.policy.step(
      frame=frame, v_ego_mps=v_ego,
      planned_accel=self.plan_accel, preview_accel=self.preview_accel,
      downstream_pid=float(accel), measured_accel=float(CS.out.aEgo),
      source=self.plan_source, has_lead=self.plan_has_lead,
      enabled=session, driver_override=False,
      feedback_275=int(getattr(CS, "hybrid_brake_request_raw_275", 0)),
      friction_08c=int(getattr(CS, "hybrid_friction_raw_08c", 0)),
      feedback_fresh=feedback_fresh, feedback_sequence=(rx275, rx08c),
      positive_accel_permitted=base_accel_agreement,
      urgent=bool(self.plan_accel <= -0.45 and float(accel) <= -0.45),
    )
    urgent = bool(self.plan_accel <= -0.45 and float(accel) <= -0.45)
    output, status = self.actuator.apply(
      base, request, enabled=session, speed_mps=v_ego,
      override=False, urgent=urgent, feedback_fresh=feedback_fresh,
      friction_active=physical_braking,
    )
    self.last_authority = status.authority
    self.last_feedback_fault = status.feedback_fault
    # Actual transmitted command veto, not a claim of physical brake torque.
    if output.brake_state in (BrakeState.BRAKING, BrakeState.CREEPING) or output.pump < 0.0:
      output.is_accel = False
      output.is_decel = True
    elif not feedback_fresh or physical_braking:
      output.is_accel = False
      output.is_decel = False
      output.speed = v_ego
    self.prev_enabled = True
    return output
