#!/usr/bin/env python3
from cereal import car, log
from selfdrive.swaglog import cloudlog
from selfdrive.config import Conversions as CV
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint, get_safety_config
from selfdrive.car.interfaces import CarInterfaceBase
from selfdrive.car.dnga.values import CAR, ACC_CAR
from selfdrive.controls.lib.desire_helper import LANE_CHANGE_SPEED_MIN
from common.params import Params

try:
  from common.features import Features
except ImportError:
  class Features:
    def has(self, feature_name):
      return False

EventName = car.CarEvent.EventName

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

    ret.lateralTuning.init('pid')
    ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
    ret.longitudinalTuning.kpV = [0.9, 0.8, 0.8]

    ret.enableGasInterceptor = False  # no physical gas interceptor
    
    # OP takes full ownership of gas and brakes again
    ret.openpilotLongitudinalControl = True

    if candidate == CAR.YARISCROSSHEV: 
      ret.wheelbase = 2.620
      ret.steerRatio = 17.00
      ret.centerToFront = ret.wheelbase * 0.44
      tire_stiffness_factor = 0.7933
      ret.mass = 1250. + STD_CARGO_KG
      ret.wheelSpeedFactor = 1.653
      ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.14], [0.32]]
      ret.lateralParams.torqueBP = [0., 10., 20., 35.]
      ret.lateralParams.torqueV  = [255, 255, 255, 255]
      ret.lateralTuning.pid.kf = 0.000188
      ret.longitudinalTuning.kpBP = [0., 5., 20.]
      ret.longitudinalTuning.kpV = [2.2, 2.0, 1.8]
      ret.longitudinalTuning.kiBP = [0.]
      ret.longitudinalTuning.kiV = [0.]
      # V2.5S: same old speed-target longcontrol architecture as Buka
      # release_ka2, so these delay bounds are directly compatible.
      # Longer look-ahead anticipates the real HEV regen/hydraulic response
      # instead of reacting after the vehicle has already overshot.
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
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront, tire_stiffness_factor=tire_stiffness_factor)

    return ret

  def update(self, c, can_strings, dragonconf):
    self.dragonconf = dragonconf

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
                rpm = int.from_bytes(dat[3:5], "big")
                self.CS.engine_rpm_raw_037 = (
                  0 if rpm == 0xFFFF else rpm
                )
      except Exception:
        pass

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
    can_sends = self.CC.update(c.enabled, c.active, self.CS, self.frame,
                               c.actuators, c.cruiseControl.cancel,
                               hud_control.visualAlert, hud_control.leftLaneVisible,
                               hud_control.rightLaneVisible, hud_control.leadVisible,
                               hud_control.leftLaneDepart, hud_control.rightLaneDepart, self.dragonconf)
    self.frame += 1
    return can_sends
