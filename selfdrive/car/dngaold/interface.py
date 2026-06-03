#!/usr/bin/env python3
from cereal import car
from selfdrive.swaglog import cloudlog
from selfdrive.config import Conversions as CV
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint, get_safety_config
from selfdrive.car.interfaces import CarInterfaceBase
from selfdrive.car.dngaold.values import CAR, ACC_CAR
from selfdrive.controls.lib.desire_helper import LANE_CHANGE_SPEED_MIN
from common.params import Params

# --- SAFE MOCK FOR BUKAPILOT'S CUSTOM FEATURES MODULE ---
try:
  from common.features import Features
except ImportError:
  class Features:
    def has(self, feature_name):
      return False

EventName = car.CarEvent.EventName

class CarInterface(CarInterfaceBase):

  # DP 0.8.13 Signature (Removed has_relay)
  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=[]):
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)
    ret.carName = "dngaold"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.toyota)]
    ret.safetyConfigs[0].safetyParam = 1
    ret.transmissionType = car.CarParams.TransmissionType.automatic
    ret.radarOffCan = True
    ret.enableApgs = False                 # advanced parking guidance system
    ret.enableDsu = False                  # driving support unit

    ret.steerRateCost = 0.7                # Lateral MPC cost on steering rate
    ret.steerLimitTimer = 0.1              # time before steerLimitAlert is issued
    ret.steerControlType = car.CarParams.SteerControlType.torque
    ret.steerActuatorDelay = 0.48          # Steering wheel actuator delay in seconds

    ret.lateralTuning.init('pid')
    ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
    ret.longitudinalTuning.kpV = [0.9, 0.8, 0.8]

    ret.enableGasInterceptor = 0x201 in fingerprint[0] or 0x401 in fingerprint[0]
    ret.openpilotLongitudinalControl = True

    f = Features()

    if candidate == CAR.YARISCROSSHEV: #YARIS CROSS NON-HYBRID BTNS FINGERPRINT
      ret.wheelbase = 2.620
      ret.steerRatio = 17.00
      ret.centerToFront = ret.wheelbase * 0.44
      tire_stiffness_factor = 0.7933
      ret.mass = 1250. + STD_CARGO_KG
      ret.wheelSpeedFactor = 1.653
      ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.16], [0.30]]
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0.], [255]]
      ret.lateralTuning.pid.kf = 0.000188
      ret.longitudinalTuning.kpBP = [0., 5., 20.]
      ret.longitudinalTuning.kpV = [2.2, 2.0, 1.8]
      ret.longitudinalTuning.kiBP = [0.]
      ret.longitudinalTuning.kiV = [0.]
      ret.longitudinalActuatorDelayLowerBound = 0.15
      ret.longitudinalActuatorDelayUpperBound = 0.20
      #

    else:
      ret.dashcamOnly = True
      ret.safetyModel = car.CarParams.SafetyModel.noOutput

    ret.minEnableSpeed = -1
    ret.steerActuatorDelay = 0.30           
    ret.enableBsm = True
    ret.stoppingDecelRate = 0.25 
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront, tire_stiffness_factor=tire_stiffness_factor)

    return ret

  # DP 0.8.13 Signature
  def update(self, c, can_strings, dragonconf):
    self.dragonconf = dragonconf
    self.cp.update_strings(can_strings)

    ret = self.CS.update(self.cp)
    ret.canValid = self.cp.can_valid
    ret.steeringRateLimited = self.CC.steer_rate_limited if self.CC is not None else False
    ret.steeringRateLimited &= self.CS.lkas_rdy

    events = self.create_common_events(ret)

    ret.events = events.to_msg()
    self.CS.out = ret.as_reader()
    return self.CS.out

  # DP 0.8.13 Signature
  def apply(self, c):
    hud_control = c.hudControl
    can_sends = self.CC.update(c.enabled, c.active, self.CS, self.frame,
                               c.actuators, c.cruiseControl.cancel,
                               hud_control.visualAlert, hud_control.leftLaneVisible,
                               hud_control.rightLaneVisible, hud_control.leadVisible,
                               hud_control.leftLaneDepart, hud_control.rightLaneDepart, self.dragonconf)
    self.frame += 1
    return can_sends
