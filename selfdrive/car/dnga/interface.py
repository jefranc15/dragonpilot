#!/usr/bin/env python3
from cereal import car, log
from selfdrive.car import (
  STD_CARGO_KG,
  scale_rot_inertia,
  scale_tire_stiffness,
  gen_empty_fingerprint,
  get_safety_config,
)
from selfdrive.car.interfaces import CarInterfaceBase
from selfdrive.car.dnga.values import CAR
from selfdrive.car.dnga.dnga_hybrid_feedback import (
  apply_hybrid_feedback_frame,
  initialize_hybrid_feedback_state,
)


def decode_engine_rpm_037(raw):
  """Decode the signed 16-bit 0x037 RPM field and clamp engine-off sentinels."""
  rpm_signed = raw if raw < 0x8000 else raw - 0x10000
  return max(0, rpm_signed)


def dnga_rx_checksum_valid(addr, dat):
  """Validate the additive DNGA checksum without trusting the DBC parser."""
  dat = bytes(dat)
  if len(dat) != 8:
    return False
  expected = (addr + len(dat[:-1]) + 1 + 2 + sum(dat[:-1])) & 0xFF
  return dat[-1] == expected


def decode_stock_acc_brake_271(dat):
  """Return a validated (state, pump, magnitude, decel) camera request."""
  dat = bytes(dat)
  if not dnga_rx_checksum_valid(0x271, dat):
    return None

  state = dat[1]
  pump_inverse = dat[3]
  pump_level = dat[4]
  magnitude = dat[5]

  if state in (0x21, 0x31, 0x30):
    if (
      dat[2] != 0x00
      or ((pump_inverse + pump_level) & 0xFF) != 0x00
      or
      # Stock also used 0.6/0.7/0.8 pump levels during the stronger 13:39
      # approach. Observe those frames but clamp controller authority to the
      # separately validated 0.87 envelope below.
      pump_level not in (4, 5, 6, 7, 8)
      or magnitude > 200
    ):
      return None
  elif state in (0x00, 0x01):
    # Neutral and stock pump-release frames are observable but never create a
    # positive brake floor in the controller.
    if magnitude != 200:
      return None
  else:
    return None

  pump = pump_level / 10.0
  decel = max(0.0, min(0.87, (200 - magnitude) / 100.0))
  return state, pump, magnitude, decel


def decode_stock_acc_cmd_273(dat):
  """Return validated camera engagement/lead/mode bits and desired speed."""
  dat = bytes(dat)
  if not dnga_rx_checksum_valid(0x273, dat):
    return None
  # Captured stock frames keep SET_ME_1_2 and SET_ME_1 asserted in both ready
  # and engaged states. Reject unrelated/corrupt payloads before exposing bits.
  if not (dat[1] & 0x02) or not (dat[6] & 0x40):
    return None

  enabled = bool(dat[1] & 0x20)
  lead = bool(dat[1] & 0x08)
  is_decel = bool(dat[4] & 0x20)
  is_accel = bool(dat[4] & 0x40)
  acc_cmd_kph = ((dat[2] << 8) | dat[3]) * 0.01
  return enabled, lead, is_accel, is_decel, acc_cmd_kph


class CarInterface(CarInterfaceBase):
  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=[]):
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)
    ret.carName = "dnga"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.toyota)]
    ret.safetyConfigs[0].safetyParam = 1
    ret.transmissionType = car.CarParams.TransmissionType.automatic
    ret.radarOffCan = True
    ret.enableApgs = False
    ret.enableDsu = False

    ret.steerRateCost = 0.7
    ret.steerLimitTimer = 0.1
    ret.steerControlType = car.CarParams.SteerControlType.torque

    ret.lateralTuning.init("pid")
    ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.0], [0.0]]
    ret.longitudinalTuning.kpV = [0.9, 0.8, 0.8]

    ret.enableGasInterceptor = False  # no physical gas interceptor

    # Software longitudinal control owns the accelerator and brake commands.
    ret.openpilotLongitudinalControl = True

    if candidate == CAR.YARISCROSSHEV:
      ret.wheelbase = 2.620
      ret.steerRatio = 17.00
      ret.centerToFront = ret.wheelbase * 0.44
      tire_stiffness_factor = 0.7933
      ret.mass = 1250.0 + STD_CARGO_KG
      ret.wheelSpeedFactor = 1.653
      ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.14], [0.32]]
      ret.lateralParams.torqueBP = [0.0, 10.0, 20.0, 35.0]
      ret.lateralParams.torqueV = [255, 255, 255, 255]
      ret.lateralTuning.pid.kf = 0.000188
      ret.longitudinalTuning.kpBP = [0.0, 5.0, 20.0]
      ret.longitudinalTuning.kpV = [2.2, 2.0, 1.8]
      ret.longitudinalTuning.kiBP = [0.0]
      ret.longitudinalTuning.kiV = [0.0]
      # Anticipate the HEV regen/hydraulic response in the speed-target controller.
      ret.longitudinalActuatorDelayLowerBound = 0.40
      ret.longitudinalActuatorDelayUpperBound = 0.50
    else:
      ret.dashcamOnly = True
      ret.safetyModel = car.CarParams.SafetyModel.noOutput

    ret.minEnableSpeed = -1
    ret.steerActuatorDelay = 0.30
    ret.enableBsm = True
    ret.stoppingDecelRate = 0.25  # Standard smooth OP stopping curve
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(
      ret.mass, ret.wheelbase, ret.centerToFront, tire_stiffness_factor=tire_stiffness_factor
    )

    return ret

  def _update_raw_can(self, can_strings):
    """Observe bus-1 hybrid feedback and validated bus-2 camera requests."""
    if not hasattr(self.CS, "hybrid_feedback_rx_frame_275"):
      initialize_hybrid_feedback_state(self.CS)

    # Read real accelerator pedal directly from raw CAN:
    # bus 1, addr 0x277, bytes 1-2 big endian.
    # These signals are on bus 1 while the main DBC parser is on bus 0.
    for can_str in can_strings:
      try:
        evt = log.Event.from_bytes(can_str)
        if evt.which() == "can":
          for msg in evt.can:
            if msg.src == 1 and msg.address == 0x277:
              dat = bytes(msg.dat)
              if len(dat) >= 3:
                self.CS.gas_raw_277 = int.from_bytes(dat[1:3], "big")

            elif msg.src == 1 and msg.address == 0x037:
              dat = bytes(msg.dat)
              if len(dat) >= 5:
                rpm_raw = int.from_bytes(dat[3:5], "big")
                self.CS.engine_rpm_raw_037 = decode_engine_rpm_037(rpm_raw)

            elif msg.src == 1 and msg.address in (
              0x08C,
              0x125,
              0x12A,
              0x275,
              0x2C9,
            ):
              # Read-only hybrid/brake feedback. Every candidate is
              # length/checksum validated before it can affect the supervisor.
              apply_hybrid_feedback_frame(self.CS, self.frame, msg.address, msg.dat)

            elif msg.src == 2 and msg.address == 0x271:
              decoded = decode_stock_acc_brake_271(msg.dat)
              if decoded is not None:
                state, pump, magnitude, decel = decoded
                self.CS.stock_acc_brake_state = state
                self.CS.stock_acc_brake_pump = pump
                self.CS.stock_brake_mag = magnitude
                self.CS.stock_acc_brake_decel = decel
                self.CS.stock_acc_brake_rx_frame = self.frame

            elif msg.src == 2 and msg.address == 0x273:
              decoded = decode_stock_acc_cmd_273(msg.dat)
              if decoded is not None:
                enabled, lead, is_accel, is_decel, acc_cmd_kph = decoded
                self.CS.stock_acc_request_enabled = enabled
                self.CS.stock_acc_request_lead = lead
                self.CS.stock_acc_request_is_accel = is_accel
                self.CS.stock_acc_request_is_decel = is_decel
                self.CS.stock_acc_cmd = acc_cmd_kph
                self.CS.stock_acc_request_rx_frame = self.frame
      except Exception:
        pass

  def update(self, c, can_strings, dragonconf):
    self.dragonconf = dragonconf

    self._update_raw_can(can_strings)

    self.cp.update_strings(can_strings)

    ret = self.CS.update(self.cp)
    ret.canValid = self.cp.can_valid
    ret.steeringRateLimited = self.CC.steer_rate_limited if self.CC is not None else False
    ret.steeringRateLimited &= self.CS.lkas_rdy

    events = self.create_common_events(ret)

    ret.events = events.to_msg()
    self.CS.out = ret.as_reader()
    return self.CS.out

  def apply(self, c):
    hud_control = c.hudControl
    can_sends = self.CC.update(
      c.enabled,
      c.active,
      self.CS,
      self.frame,
      c.actuators,
      c.cruiseControl.cancel,
      hud_control.visualAlert,
      hud_control.leftLaneVisible,
      hud_control.rightLaneVisible,
      hud_control.leadVisible,
      hud_control.leftLaneDepart,
      hud_control.rightLaneDepart,
      self.dragonconf,
    )
    self.frame += 1
    return can_sends
