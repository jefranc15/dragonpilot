"""DNGA V7.1 OEM-follow shadow controller (factory longitudinal unchanged).

Toyota's original camera 0x271/0x273/0x274 messages are forwarded by the
matching dedicated Panda safety hook. OP transmits only 0x1D0 steering.
Factory brake/pump and OP vision-turn proposals are shadow only: never sent.
"""

from cereal import car, messaging
from common.numpy_fast import clip, interp
from opendbc.can.packer import CANPacker
from selfdrive.car.dnga.dngacan import create_can_steer_command
from selfdrive.car.dnga.stock_long_shadow import StockBrakeShadow
from selfdrive.car.dnga.oem_follow_shadow import OEMFollowShadow, CameraSnapshot, TurnSnapshot
from selfdrive.car.dnga.values import DBC, CarControllerParams

try:
  from selfdrive.swaglog import cloudlog
except ImportError:
  cloudlog = None


def apply_dnga_steer_torque_limits(apply_torque, apply_torque_last, driver_torque, blinker_on, LIMITS):
  reduced_torque_mult = 10 if blinker_on else 1.5
  driver_max_torque = 255 + driver_torque * reduced_torque_mult
  driver_min_torque = -255 - driver_torque * reduced_torque_mult
  max_steer_allowed = clip(driver_max_torque, 0, 255)
  min_steer_allowed = clip(driver_min_torque, -255, 0)
  apply_torque = clip(apply_torque, min_steer_allowed, max_steer_allowed)
  if apply_torque_last > 0:
    apply_torque = clip(
      apply_torque,
      max(apply_torque_last - LIMITS.STEER_DELTA_DOWN, -LIMITS.STEER_DELTA_UP),
      apply_torque_last + LIMITS.STEER_DELTA_UP,
    )
  else:
    apply_torque = clip(
      apply_torque,
      apply_torque_last - LIMITS.STEER_DELTA_UP,
      min(apply_torque_last + LIMITS.STEER_DELTA_DOWN, LIMITS.STEER_DELTA_UP),
    )
  return int(round(float(apply_torque)))


class CarController:
  def __init__(self, dbc_name, CP, VM):
    self.last_steer = 0
    self.steer_rate_limited = False
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(DBC[CP.carFingerprint]["pt"])
    self.stock_ldw = False
    self.stock_brake_shadow = StockBrakeShadow()
    self.oem_follow_shadow = OEMFollowShadow()
    self.last_shadow_mode = ""
    self.turn_plan = TurnSnapshot()
    try:
      # Read-only longitudinalPlan subscription; missing input means OEM-only.
      self.turn_plan_sm = messaging.SubMaster(["longitudinalPlan"])
    except Exception:
      self.turn_plan_sm = None

    # Never silently use this controller with OP longitudinal enabled:
    # its sole-owner camera forwarding needs a different Panda safety model.
    if CP.openpilotLongitudinalControl or CP.safetyConfigs[0].safetyModel != car.CarParams.SafetyModel.dngaStock:
      raise RuntimeError("V7 stocklong requires factory longitudinal and dngaStock Panda safety")

  def _read_turn_plan(self, frame):
    """Observe OP turn planning; no planner output ever reaches CAN."""
    if self.turn_plan_sm is not None:
      try:
        self.turn_plan_sm.update(0)
        if self.turn_plan_sm.updated["longitudinalPlan"]:
          plan = self.turn_plan_sm["longitudinalPlan"]
          accels = list(getattr(plan, "accels", []))
          self.turn_plan = TurnSnapshot(
            rx_frame=frame,
            source=str(getattr(plan, "longitudinalPlanSource", "")),
            has_lead=bool(getattr(plan, "hasLead", False)),
            turn_speed_mps=float(getattr(plan, "visionTurnSpeed", 0.0)),
            near_accel=float(accels[0]) if accels else 0.0,
            lookahead_accel=min(float(v) for v in accels[:13]) if accels else 0.0,
          )
      except Exception:
        # Stale plan is detected by OEMFollowShadow; never synthesize curves.
        self.turn_plan_sm = None
    return self.turn_plan

  def update(
    self, enabled, active, CS, frame, actuators, pcm_cancel_cmd,
    hud_alert, left_line, right_line, lead, left_lane_depart,
    right_lane_depart, dragonconf,
  ):
    can_sends = []

    steer_max_interp = max(1.0, interp(CS.out.vEgo, self.params.STEER_BP, self.params.STEER_LIM_TORQ))
    new_steer = int(round(actuators.steer * steer_max_interp))
    blinker_on = CS.out.leftBlinker != CS.out.rightBlinker
    apply_steer = apply_dnga_steer_torque_limits(
      new_steer, self.last_steer, CS.out.steeringTorqueEps, blinker_on, self.params
    )
    self.steer_rate_limited = new_steer != apply_steer and apply_steer != 0
    self.steer_rate_limited &= not CS.out.steeringPressed

    if frame % CarControllerParams.STEER_STEP == 0:
      steer_req = (enabled or self.stock_ldw) and CS.lkas_latch
      can_sends.append(
        create_can_steer_command(
          self.packer, apply_steer, steer_req,
          frame // CarControllerParams.STEER_STEP % 16
        )
      )

    # Observe the OEM camera request; never feed the proposal to the ECU.
    if frame % CarControllerParams.ACC_STEP == 0:
      observation = self.stock_brake_shadow.observe(
        frame,
        getattr(CS, "stock_acc_brake_rx_frame", -1000000),
        getattr(CS, "stock_acc_brake_state", -1),
        getattr(CS, "stock_brake_mag", 200),
      )
      camera = CameraSnapshot(
        brake_rx_frame=getattr(CS, "stock_acc_brake_rx_frame", -1000000),
        acc_rx_frame=getattr(CS, "stock_acc_request_rx_frame", -1000000),
        brake_state=getattr(CS, "stock_acc_brake_state", -1),
        brake_magnitude=getattr(CS, "stock_brake_mag", 200),
        pump=getattr(CS, "stock_acc_brake_pump", 0.0),
        enabled=bool(getattr(CS, "stock_acc_request_enabled", False)),
        lead=bool(getattr(CS, "stock_acc_request_lead", False)),
        is_accel=bool(getattr(CS, "stock_acc_request_is_accel", False)),
        is_decel=bool(getattr(CS, "stock_acc_request_is_decel", False)),
        desired_speed_kph=getattr(CS, "stock_acc_cmd", 0.0),
        set_speed_kph=getattr(CS, "stock_acc_set_speed", 0.0),
      )
      assessment = self.oem_follow_shadow.observe(
        frame, camera, self._read_turn_plan(frame),
        ego_speed_mps=CS.out.vEgo,
        ego_accel_mps2=CS.out.aEgo,
        gas_pressed=CS.out.gasPressed,
        brake_pressed=CS.out.brakePressed,
        brake_shadow=observation,
      )
      # OEM commands and shadow recommendations must not change can_sends.
      if cloudlog is not None and (frame % 100 == 0 or assessment.mode != self.last_shadow_mode):
        cloudlog.info(
          "V71_OEM_FOLLOW_SHADOW mode=%s reason=%s lead=%d OEM_accel=%d OEM_decel=%d "
          "OEM_target=%.1f hypothetical_turn_target=%.1f OEM_brake=%.3f "
          "hypothetical_brake=%.3f OEM_pump=%.2f hypothetical_pump=%.2f "
          "weakening=%.2f (NOT TRANSMITTED)",
          assessment.mode, assessment.reason,
          assessment.camera_lead, assessment.camera_acceleration, assessment.camera_deceleration,
          assessment.factory_acc_kph, assessment.hypothetical_curve_target_kph,
          assessment.camera_brake_strength, assessment.hypothetical_brake_strength,
          assessment.camera_pump, assessment.hypothetical_pump, assessment.weakening_fraction,
        )
      self.last_shadow_mode = assessment.mode

    # This is intentionally the complete transmit list. Stock ACC 0x271/0x273
    # and stock HUD 0x274 remain entirely OEM-owned by the dedicated Panda
    # forwarding hook. No OP curve brake, no synthetic lead, no ACC speed
    # target, no pump override, no active brake magnitude scaling.
    self.last_steer = apply_steer
    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / steer_max_interp
    new_actuators.accel = 0.0
    return new_actuators, can_sends
