"""DNGA v6.0 R2 moving brake policy, with native V5.0 scaling and SNG.

This is a single stateful controller. It inherits the existing DNGA
longitudinal session/lead/stop-and-go/handoff handling; only the moving
hydraulic decision changes. The v5.0 coast-first arbitration is NOT invoked
while R2 owns moving braking.

OFFLINE VALIDATION ONLY: no guarantee of physical braking, regen, or torque.
"""
from common.numpy_fast import clip, interp
from selfdrive.config import Conversions as CV
from selfdrive.car.dnga.longitudinal_v50_legacy import (
  BrakeRequest, curve_brake_cap, distance_profile,
  high_speed_brake_cap, low_speed_brake_cap, progressive_brake_cap,
  stop_guard_brake_cap,
)
from selfdrive.car.dnga.longitudinal_kinematic_guard import LongitudinalController as V50LongitudinalController
from selfdrive.car.dnga.values import BrakeState, LongitudinalParams as P

R2_MOVING_MIN_SPEED = P.CURVE_HYDRAULIC_MIN_SPEED  # 5.6 m/s; below this use unmodified V5.0 SNG.
R2_FIRST_BRAKE = 0.08                        # Initial ordinary BRAKING = 0x04C0 (1216).
R2_SOURCE_BRIDGE_FRAMES = 50                # Maximum 0.50 s at 100 Hz.
R2_RELEASE_FRAMES = 120                     # 24 x 20 Hz READY frames (not validated in ECU).
R2_TARGET_GAP_MAX_KPH = 10.7
R2_GAP_STEP_UP_KPH = 0.18
R2_GAP_STEP_DOWN_KPH = 0.35
# Fitted to checksum-valid OEM bus-2 0x271/0x273 and CarState observations.
# 1,047 BRAKING frames (8 Nov + 4 older rlogs), 8-80 km/h, 0.08-0.87
# encoded strength. Mean absolute OEM ACC gap fit error: ~0.221 km/h.
R2_STOCK_GAP_INTERCEPT = -0.31592604
R2_STOCK_GAP_STRENGTH = 7.01016383
R2_STOCK_GAP_SPEED = 0.03393986
R2_STOCK_GAP_MIN_SPEED_KPH = 8.0
R2_STOCK_GAP_MAX_SPEED_KPH = 80.0
# Gentler normal mode: 85% of the OEM commanded speed offset.
# Preserve the full fitted offset for urgent/critical braking.
R2_COMFORT_GAP_FACTOR = 0.85
# On positive OEM 0x271 strength changes, median step was 0.02 per
# 50-ms message. Use 75% of that (+0.015) for R2 normal comfort ramps.
# Original V5.0 filter, governor, caps and urgent escalation remain in force.
R2_STOCK_NORMAL_STEP = 0.020
R2_COMFORT_NORMAL_STEP = 0.75 * R2_STOCK_NORMAL_STEP


def r2_brake_cap(v_ego, *, curve, urgent, guard_active, lead_cap):
  """Reuse the V5.0 speed/lead/curve/stop-guard braking envelopes."""
  if curve:
    return curve_brake_cap(v_ego)
  if guard_active:
    return stop_guard_brake_cap(v_ego)
  if urgent:
    return min(P.STOP_BRAKE_MAX, P.URGENT_BRAKE_MAX,
               low_speed_brake_cap(v_ego), high_speed_brake_cap(v_ego))
  cap = min(lead_cap, progressive_brake_cap(v_ego)) if v_ego >= P.HIGHWAY_MIN_SPEED else lead_cap
  return min(cap, low_speed_brake_cap(v_ego), high_speed_brake_cap(v_ego))


def r2_stock_acc_gap_kph(v_kph, brake_strength):
  """OEM command fit; not actual braking acceleration or regen torque.

  Use only the sampled speed/strength envelope; clamp rather than extrapolate.
  No new hydraulic authority or physical-response estimate is inferred.
  """
  v_kph = max(0.0, float(v_kph))
  if v_kph <= 0.0:
    return 0.0
  fit_speed = clip(v_kph, R2_STOCK_GAP_MIN_SPEED_KPH, R2_STOCK_GAP_MAX_SPEED_KPH)
  fit_strength = clip(float(brake_strength), 0.08, P.STOP_BRAKE_MAX)
  return max(0.0, min(v_kph, R2_TARGET_GAP_MAX_KPH,
                      R2_STOCK_GAP_INTERCEPT +
                      R2_STOCK_GAP_STRENGTH * fit_strength +
                      R2_STOCK_GAP_SPEED * fit_speed))


def r2_acc_gap_kph(v_kph, brake_strength, urgent=False):
  """Milder normal 0x273 target; never attenuate urgent/critical cases."""
  oem_gap = r2_stock_acc_gap_kph(v_kph, brake_strength)
  factor = 1.0 if urgent else R2_COMFORT_GAP_FACTOR
  return max(0.0, min(float(v_kph), R2_TARGET_GAP_MAX_KPH, factor * oem_gap))


class LongitudinalController(V50LongitudinalController):
  """Replace only normal moving-brake arbitration; preserve one V5.0 SNG state."""

  def __init__(self):
    super().__init__()

  def _reset(self, frame, a_ego):
    super()._reset(frame, a_ego)
    self.r2_moving = False
    self.r2_release_until_frame = frame
    self.r2_gap_kph = 0.0
    self.r2_bridge_since_frame = -1000000
    self.r2_last_authorized_source = ""
    self.r2_last_turn_speed = 0.0
    self.r2_authority = "legacy"

  def _update_hydraulic(self, CS, frame, session, apply_accel, plan, lead_state):
    v = max(0.0, float(CS.out.vEgo))
    # All V5.0 SNG, CREEPING, HOLD, and low-speed actuator transitions use
    # the SAME inherited controller instance and its continuously maintained
    # lead, brake and hold state. No secondary low-speed controller.
    sng_domain = bool(v < R2_MOVING_MIN_SPEED or CS.out.standstill or
                      self.sng_armed or self.stop_hold or self.sng_release_count > 0)
    if sng_domain or not session.allowed:
      self.r2_moving = False
      self.r2_authority = "v50_sng_or_inactive"
      self.r2_bridge_since_frame = -1000000
      self.r2_release_until_frame = frame
      self.r2_gap_kph = 0.0
      return super()._update_hydraulic(CS, frame, session, apply_accel, plan, lead_state)

    # R2 no longer consults V5.0's moving lead->coast->hydraulic gate, coast
    # handoff counter, normal reentry timer, or independent R2 shadow object.
    lead_allowed = bool(lead_state.relevant and lead_state.status and plan.has_lead)
    curve_allowed = bool(plan.curve_active and
                         (plan.turn_speed <= 0.0 or
                          v > plan.turn_speed + P.CURVE_SPEED_ERROR_RELEASE))
    set_speed_allowed = bool(plan.source == "cruise" and not plan.has_lead and
                             v > CS.out.cruiseState.speed + P.CRUISE_DECEL_MARGIN)
    authorized = bool(plan.fresh and (lead_allowed or curve_allowed or set_speed_allowed))

    bridge = False
    if authorized:
      self.r2_bridge_since_frame = -1000000
      self.r2_last_authorized_source = plan.source
      if curve_allowed:
        self.r2_last_turn_speed = plan.turn_speed
    elif (plan.fresh and self.r2_moving and
          self.r2_last_authorized_source in ("turn", "lead0", "lead1") and
          plan.source == "cruise" and
          plan.brake >= 0.14 and apply_accel <= -0.05 and
          float(CS.out.aEgo) >= plan.accel - 0.18 and
          (self.r2_last_authorized_source != "turn" or
           self.r2_last_turn_speed <= 0.0 or
           v > self.r2_last_turn_speed + P.CURVE_SPEED_ERROR_RELEASE)):
      if self.r2_bridge_since_frame < 0:
        self.r2_bridge_since_frame = frame
      bridge = frame - self.r2_bridge_since_frame <= R2_SOURCE_BRIDGE_FRAMES
    else:
      self.r2_bridge_since_frame = -1000000

    urgent = bool(
      lead_state.urgent or
      (plan.fresh and plan.brake >= P.URGENT_HARD_DECEL and apply_accel < -0.05)
    )
    emergency = bool(
      lead_state.status and lead_state.closing_speed >= P.EMERGENCY_CLOSING_SPEED and
      lead_state.ttc <= P.EMERGENCY_TTC and plan.brake >= P.EMERGENCY_PLANNER_BRAKE
    )
    critical = bool(lead_state.status and
                    lead_state.closing_speed >= P.DECEL_GOVERNOR_CRITICAL_CLOSING and
                    lead_state.ttc <= P.DECEL_GOVERNOR_CRITICAL_TTC)

    raw_demand = plan.brake
    if lead_allowed:
      # V5.0's bounded lead lookahead remains, with the same closing/TTC gate.
      if lead_state.closing_speed >= P.LEAD_LOOKAHEAD_MIN_CLOSING and lead_state.ttc <= P.LEAD_LOOKAHEAD_MAX_TTC:
        raw_demand = max(raw_demand,
                         min(P.LEAD_LOOKAHEAD_TARGET_CAP, plan.lead_lookahead_brake))
      if apply_accel < 0.0:
        pid_extra = max(0.0, -apply_accel - plan.brake)
        raw_demand += min(P.PID_BRAKE_ALLOWANCE, P.PID_BRAKE_BLEND * pid_extra)
    demand = raw_demand if authorized or bridge else 0.0
    need_brake = bool(demand > P.DECEL_DEADBAND)

    # Keep the existing independent low-speed stop-guard tracking current even
    # during moving R2 operation so the transition into the inherited SNG
    # regime does not begin with stale geometry counters.
    self.filtered_aego += P.AEGO_FILTER_ALPHA * (float(CS.out.aEgo) - self.filtered_aego)
    guard = self._update_stop_guard(CS, frame, plan.brake, True, lead_state)
    distance_val = int(clip(getattr(CS, "op_distance_val", 1), 0, 2))
    lead_cap = distance_profile(distance_val)[1]

    if need_brake:
      if not self.brake_active:
        # Only the *first normal active request* must be 1216. Subsequent
        # strength is computed by V5.0's filters, slew limits and speed caps.
        self.brake_active = True
        self.curve_brake_active = bool(curve_allowed and not lead_allowed)
        self.urgent_brake = urgent
        self.apply_brake = R2_FIRST_BRAKE
        self.brake_target = R2_FIRST_BRAKE
        self.release_counter = 0
        self.r2_release_until_frame = frame
      else:
        self.curve_brake_active = bool(curve_allowed and not lead_allowed)
        self.urgent_brake = urgent
        cap = r2_brake_cap(v, curve=self.curve_brake_active,
                           urgent=urgent or emergency, guard_active=guard.active,
                           lead_cap=lead_cap)
        scale = interp(v, [0.0, 140.0 * CV.KPH_TO_MS], [1.0, 1.0 / 1.5])
        brake_floor = R2_FIRST_BRAKE
        requested = float(clip(max(demand, guard.request if guard.active else 0.0) *
                               scale, brake_floor, max(brake_floor, cap)))
        previous_brake = self.apply_brake
        self._apply_brake_target(
          requested, brake_floor, max(brake_floor, cap),
          emergency, critical, guard,
        )
        # The V5.0 filtered target and speed/lead caps still decide HOW MUCH
        # braking is justified. Only the normal positive slew is reshaped:
        # OEM median positive delta 0.020/50ms, comfort delta 0.015/50ms.
        # Do not bypass the measured-deceleration governor, stop guard,
        # urgent/emergency ramps or the original downward slew.
        normal_comfort = (
          not (urgent or emergency or critical or guard.active)
          and self.filtered_aego > P.DECEL_GOVERNOR_START
          and self.brake_target > previous_brake
          and self.apply_brake > previous_brake
        )
        if normal_comfort:
          self.apply_brake = min(
            max(brake_floor, cap), self.brake_target,
            previous_brake + R2_COMFORT_NORMAL_STEP,
          )
      self.r2_moving = True
      self.r2_authority = "r2_braking"
      self.handoff_pending = True
      self.handoff_ready_counter = 0
      self.kinematic_ready_counter = 0
      return BrakeRequest(True, False, False)

    if self.r2_moving and self.brake_active:
      # Use the V5.0 physical brake ramp-down instead of an instant 0x271 off.
      self.release_counter += 1
      self.apply_brake = max(R2_FIRST_BRAKE, self.apply_brake - P.BRAKE_STEP_DOWN)
      self.brake_target = R2_FIRST_BRAKE
      if self.release_counter >= P.RELEASE_CONFIRM_COUNT and self.apply_brake <= R2_FIRST_BRAKE:
        self._clear_hydraulic(frame, reentry=False)
        self.r2_moving = False
        self.r2_release_until_frame = frame + R2_RELEASE_FRAMES
        self.handoff_pending = True
        self.handoff_ready_counter = 0
        self.kinematic_ready_counter = 0
      else:
        self.r2_authority = "r2_brake_ramp_down"
        return BrakeRequest(True, False, False)

    release = bool(frame < self.r2_release_until_frame)
    self.r2_authority = "r2_ready_release" if release else "r2_ready"
    return BrakeRequest(False, release, False)

  def _build_command(self, CS, frame, session, plan, lead_state, brake, propulsion):
    # The base computes ALL existing V5.0 SNG, emergency/session and
    # positive-accel gating. Only change the ACC_CMD target on R2-owned
    # moving BRAKING or explicit R2 READY release frames.
    command = super()._build_command(CS, frame, session, plan, lead_state, brake, propulsion)
    moving_r2 = bool(session.allowed and not CS.out.standstill and
                     CS.out.vEgo >= R2_MOVING_MIN_SPEED and
                     (self.r2_moving or (brake.release_pump and frame < self.r2_release_until_frame)))
    if moving_r2:
      v_kph = max(0.0, float(CS.out.vEgo) * 3.6)
      target = (r2_acc_gap_kph(v_kph, self.apply_brake,
                               urgent=bool(self.urgent_brake or lead_state.urgent))
                if brake.hydraulic else 0.0)
      change = clip(target - self.r2_gap_kph,
                    -R2_GAP_STEP_DOWN_KPH, R2_GAP_STEP_UP_KPH)
      self.r2_gap_kph = clip(self.r2_gap_kph + change, 0.0, v_kph)
      if command.brake_state == BrakeState.BRAKING or brake.release_pump:
        command.speed = max(0.0, CS.out.vEgo - self.r2_gap_kph / 3.6)
        command.is_accel = False
        command.is_decel = True
    else:
      self.r2_gap_kph = 0.0
    return command
