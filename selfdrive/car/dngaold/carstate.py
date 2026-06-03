from cereal import car
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from common.numpy_fast import mean, interp, clip
from selfdrive.config import Conversions as CV
from selfdrive.car.interfaces import CarStateBase
from selfdrive.car.dngaold.values import DBC, CAR, ACC_CAR, HUD_MULTIPLIER
from time import time

# todo: clean this part up
pedal_counter = 0
pedal_press_state = 0
PEDAL_COUNTER_THRES = 35
PEDAL_UPPER_TRIG_THRES = 0.125
PEDAL_NON_ZERO_THRES = 0.01

SEC_HOLD_TO_STEP_SPEED = 0.6

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    self.shifter_values = can_define.dv["TRANSMISSION"]['GEAR']
    if CP.carFingerprint in ACC_CAR:
      self.set_distance_values = can_define.dv['ACC_CMD_HUD']['FOLLOW_DISTANCE']
    self.is_cruise_latch = False
    self.cruise_speed = 30 * CV.KPH_TO_MS
    self.cruise_speed_counter = 0
    self.acttrGas = 0

    self.is_plus_btn_latch = False
    self.is_minus_btn_latch = False
    self.prev_distance_btn = False
    self.op_distance_val = 2

    # shared by both + and - button, since release of another button will reset this
    self.rising_edge_since = 0
    self.last_frame = time() # todo: existing infra to reuse?
    self.dt = 0

    self.stock_lkc_off = True
    self.stock_fcw_off = True
    self.lkas_rdy = True
    self.lkas_latch = True # Set LKAS for Perodua to True by default
    self.lkas_btn_rising_edge_seen = False
    self.stock_acc_engaged = False
    self.stock_acc_cmd = 0
    self.stock_brake_mag = 0
    self.stock_acc_set_speed = 0

  def update(self, cp):
    ret = car.CarState.new_message()

#    ret.lkaDisabled = not self.lkas_latch

    # there is a backwheel speed, but it will overflow to 0 when reach 60kmh
    # perodua vehicles doesn't have a good standard for their wheelspeed scaling
    #WHEEL_FACTOR = 1.132
    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["WHEEL_SPEED"]['WHEELSPEED_F'],# * WHEEL_FACTOR,
      cp.vl["WHEEL_SPEED"]['WHEELSPEED_F'],# * WHEEL_FACTOR,
      cp.vl["WHEEL_SPEED"]['WHEELSPEED_F'],# * WHEEL_FACTOR,
      cp.vl["WHEEL_SPEED"]['WHEELSPEED_F'],# * WHEEL_FACTOR,
    )
    ret.vEgoRaw = mean([ret.wheelSpeeds.rr, ret.wheelSpeeds.rl, ret.wheelSpeeds.fr, ret.wheelSpeeds.fl])
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.01

    # safety checks to engage
    can_gear = int(cp.vl["TRANSMISSION"]['GEAR'])

    ret.doorOpen = any([cp.vl["METER_CLUSTER"]['MAIN_DOOR'],
                     cp.vl["METER_CLUSTER"]['LEFT_FRONT_DOOR'],
                     cp.vl["METER_CLUSTER"]['RIGHT_BACK_DOOR'],
                     cp.vl["METER_CLUSTER"]['LEFT_BACK_DOOR']])

    ret.seatbeltUnlatched = cp.vl["METER_CLUSTER"]['SEAT_BELT_WARNING'] == 1
    #if self.CP.carFingerprint in ACC_CAR:
    ret.seatbeltUnlatched |= cp.vl["METER_CLUSTER"]['SEAT_BELT_WARNING2'] == 1
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))

    self.is_cruise_latch = False if (ret.doorOpen or ret.seatbeltUnlatched) else self.is_cruise_latch

    # gas pedal
    ret.gas = cp.vl["GAS_PEDAL"]['APPS_1']
    # todo: let gas pressed legit
    ret.gasPressed = not bool(cp.vl["GAS_PEDAL_2"]['GAS_PEDAL_STEP'])
    self.acttrGas = (cp.vl["GAS_SENSOR"]['INTERCEPTOR_GAS']) # KommuActuator gas, read when stock pedal is being intercepted
    if self.acttrGas < 0:
      self.acttrGas = 0

    # brake pedal
    ret.brake = cp.vl["BRAKE"]['BRAKE_PRESSURE']
    ret.brakePressed = bool(cp.vl["BRAKE"]['BRAKE_ENGAGED'])

    # steer
    ret.steeringAngleDeg = cp.vl["STEERING_MODULE"]['STEER_ANGLE']
    ret.steeringTorque = cp.vl["STEERING_MODULE"]['MAIN_TORQUE']
    ret.steeringTorqueEps = cp.vl["EPS_SHAFT_TORQUE"]['STEERING_TORQUE']
    ret.steeringPressed = bool(abs(ret.steeringTorque) > 20)
    ret.steerWarning = False
    ret.steerError = False

    

    v_ego_cluster = cp.vl["BUTTONS"]["UI_SPEED"] * CV.KPH_TO_MS * HUD_MULTIPLIER
    self.stock_adas_frontDepartureHUD = bool(cp.vl["LKAS_HUD"]["FRONT_DEPART"])
    self.stock_adas_aebV = cp.vl ["ACC_BRAKE"]['AEB_1019']
    self.stock_aeb = bool(cp.vl["LKAS_HUD"]['AEB_BRAKE'])
    self.stock_fcw = bool(cp.vl["LKAS_HUD"]['AEB_ALARM'])
    self.stock_lkc_off = bool(cp.vl["LKAS_HUD"]['LDA_OFF'])
    self.lkas_rdy = bool(cp.vl["LKAS_HUD"]['LKAS_SET'])
    self.stock_fcw_off = bool(cp.vl["LKAS_HUD"]['FCW_DISABLE'])

    self.stock_acc_cmd = cp.vl["ACC_CMD_HUD"]["ACC_CMD"] # kph
    self.stock_acc_engaged = self.stock_acc_cmd > 0
    self.stock_acc_set_speed = cp.vl["ACC_CMD_HUD"]["SET_SPEED"] #kph
    self.stock_brake_mag = -1 * cp.vl["ACC_BRAKE"]["MAGNITUDE"]
      

    # logic to engage LKC
    if bool(cp.vl["BUTTONS"]['LKC_BTN']):
      if not self.lkas_btn_rising_edge_seen:
        self.lkas_btn_rising_edge_seen = True

    if self.lkas_btn_rising_edge_seen and not bool(cp.vl["BUTTONS"]['LKC_BTN']):
      self.lkas_latch = not self.lkas_latch
      self.lkas_btn_rising_edge_seen = False

    ret.cruiseState.available = True

             # Read the button
    distance_btn = cp.vl["BUTTONS"]["DISTANCE_BTN"]
      
    if distance_btn and not self.prev_distance_btn:
        # Cycle the custom distance value (2 -> 1 -> 0 -> 2)
        self.op_distance_val -= 1
        if self.op_distance_val < 0:
            self.op_distance_val = 2
              
        # Fire the event for the AI
        be = car.CarState.ButtonEvent.new_message()
        be.type = car.CarState.ButtonEvent.Type.gapAdjustCruise
        be.pressed = True
          
        events = list(ret.buttonEvents)
        events.append(be)
        ret.buttonEvents = events
          
    self.prev_distance_btn = distance_btn

    # todo: check if the logic needs to be this complicated
    minus_button = bool(cp.vl["PCM_BUTTONS"]["SET_MINUS"])
    plus_button = bool(cp.vl["PCM_BUTTONS"]["RES_PLUS"])

    if self.is_cruise_latch:
      cur_time = time()
      self.dt += cur_time - self.last_frame
      self.last_frame = cur_time

      if self.is_plus_btn_latch != plus_button: # rising or falling
        if not plus_button: # released, falling
          if cur_time - self.rising_edge_since < 1:
              self.cruise_speed += CV.KPH_TO_MS
        else: # pressed, rising, init
          self.rising_edge_since = cur_time
          self.dt = 0
      elif plus_button: # is holding
        while self.dt >= SEC_HOLD_TO_STEP_SPEED:
          kph = self.cruise_speed * CV.MS_TO_KPH
          kph += 5 - (kph % 5)  # step up to next nearest 5
          self.cruise_speed = kph * CV.KPH_TO_MS
          self.dt -= SEC_HOLD_TO_STEP_SPEED

      if self.is_minus_btn_latch != minus_button: # rising or falling
        if not minus_button: # released, falling
          if cur_time - self.rising_edge_since < 1:
            self.cruise_speed -= CV.KPH_TO_MS
        else: # pressed, rising
          self.rising_edge_since = cur_time
          self.dt = 0
      elif minus_button: # is holding
        while self.dt >= SEC_HOLD_TO_STEP_SPEED:
          kph = self.cruise_speed * CV.MS_TO_KPH
          kph = ((kph / 5) - 1) * 5  # step down to next nearest 5
          kph = max(30, kph)
          self.cruise_speed = kph * CV.KPH_TO_MS
          self.dt -= SEC_HOLD_TO_STEP_SPEED

    if not self.is_cruise_latch:
      # activate cruise onReleased
      if self.is_plus_btn_latch and not plus_button:
        self.is_cruise_latch = True

      elif self.is_minus_btn_latch and not minus_button:
        self.cruise_speed = max(30 * CV.KPH_TO_MS, v_ego_cluster)
        self.is_cruise_latch = True


    self.is_plus_btn_latch = plus_button
    self.is_minus_btn_latch = minus_button

    if bool(cp.vl["PCM_BUTTONS"]["CANCEL"]):
      self.is_cruise_latch = False

    if ret.brakePressed:
      self.is_cruise_latch = False

    # set speed in range of 30 - 125kmh only
    self.cruise_speed = clip(self.cruise_speed, 30 * CV.KPH_TO_MS, 125 * CV.KPH_TO_MS)
    ret.cruiseState.speed = self.cruise_speed

    ret.cruiseState.standstill = False
    ret.cruiseState.nonAdaptive = False
    ret.cruiseState.enabled = self.is_cruise_latch
    if not ret.cruiseState.available:
      self.is_cruise_latch = False

    # button presses
    ret.leftBlinker = bool(cp.vl["METER_CLUSTER"]["LEFT_SIGNAL"])
    ret.rightBlinker = bool(cp.vl["METER_CLUSTER"]["RIGHT_SIGNAL"])
    ret.genericToggle = bool(cp.vl["RIGHT_STALK"]["GENERIC_TOGGLE"])

    # blindspot sensors
    if self.CP.enableBsm:
      # used for lane change so its okay for the chime to work on both side.
      ret.leftBlindspot = bool(cp.vl["BSM"]["BSM_CHIME"])
      ret.rightBlindspot = bool(cp.vl["BSM"]["BSM_CHIME"])
    else:
      ret.leftBlindspot = False
      ret.rightBlindspot = False

    return ret

  @staticmethod
  def check_pedal_engage(gas,state):
    ''' Pedal engage logic '''
    global pedal_counter
    global pedal_press_state
    if (state == 0):
      if (gas > PEDAL_UPPER_TRIG_THRES):
        pedal_counter += 1
        if (pedal_counter == PEDAL_COUNTER_THRES):
          pedal_counter = 0
          return False
      if (pedal_counter > 2 and gas <= PEDAL_NON_ZERO_THRES):
        pedal_press_state = 1
        pedal_counter = 0
      return False
    if (state == 1):
      pedal_counter += 1
      if (pedal_counter == PEDAL_COUNTER_THRES):
        pedal_counter = 0
        pedal_press_state = 0
        return False
      if (gas > PEDAL_UPPER_TRIG_THRES):
        pedal_press_state = 2
        pedal_counter = 0
      return False
    if (state == 2):
      pedal_counter += 1
      if (pedal_counter == PEDAL_COUNTER_THRES):
        pedal_counter = 0
        pedal_press_state = 0
        return False
      if (gas <= PEDAL_NON_ZERO_THRES):
        pedal_counter = 0
        pedal_press_state = 0
        return True
    return False


  @staticmethod
  def get_can_parser(CP):
    import os
    signals = [
      ("WHEELSPEED_F", "WHEEL_SPEED", 0.),
      ("GEAR", "TRANSMISSION", 0),
      ("APPS_1", "GAS_PEDAL", 0.),
      ("BRAKE_PRESSURE", "BRAKE", 0.),
      ("BRAKE_ENGAGED", "BRAKE", 0),
      ("INTERCEPTOR_GAS", "GAS_SENSOR", 0),
      ("GENERIC_TOGGLE", "RIGHT_STALK", 0),
      ("LEFT_SIGNAL", "METER_CLUSTER", 0),
      ("RIGHT_SIGNAL", "METER_CLUSTER", 0),
      ("SEAT_BELT_WARNING", "METER_CLUSTER", 0),
      ("MAIN_DOOR", "METER_CLUSTER", 1),
      ("LEFT_FRONT_DOOR", "METER_CLUSTER", 1),
      ("RIGHT_BACK_DOOR", "METER_CLUSTER", 1),
      ("LEFT_BACK_DOOR", "METER_CLUSTER", 1)
    ]

    if CP.carFingerprint in ACC_CAR:
      signals += [
        ("BSM_CHIME","BSM", 0),
        ("SEAT_BELT_WARNING2","METER_CLUSTER", 0),
        ("STEER_ANGLE", "STEERING_MODULE", 0.),
        ("MAIN_TORQUE", "STEERING_MODULE", 0.),
        ("STEERING_TORQUE", "EPS_SHAFT_TORQUE", 0.),
        ("ACC_RDY", "PCM_BUTTONS", 0),
        ("GAS_PRESSED", "PCM_BUTTONS_HYBRID", 0),
        ("SET_MINUS", "PCM_BUTTONS", 0),
        ("SET_MINUS", "PCM_BUTTONS_HYBRID", 0),
        ("RES_PLUS", "PCM_BUTTONS_HYBRID", 0),
        ("CANCEL", "PCM_BUTTONS_HYBRID", 0),
        ("RES_PLUS","PCM_BUTTONS", 0),
        ("CANCEL","PCM_BUTTONS", 0),
        ("PEDAL_DEPRESSED","PCM_BUTTONS", 0),
        ("LKAS_ENGAGED", "LKAS_HUD", 0),
        ("LDA_OFF", "LKAS_HUD", 0),
        ("FCW_DISABLE", "LKAS_HUD", 0),
        ("LDA_RELATED1", "LKAS_HUD", 0),
        ("LDA_ALERT", "LKAS_HUD", 0),
        ("LKAS_SET", "LKAS_HUD", 0),
        ("ACC_CMD", "ACC_CMD_HUD", 0),
        ("SET_ME_1_2", "ACC_CMD_HUD", 0),
        ("STEER_CMD", "STEERING_LKAS", 0),
        ("STEER_REQ", "STEERING_LKAS", 0),
        ("FRONT_DEPART", "LKAS_HUD", 0),
        ("AEB_BRAKE", "LKAS_HUD", 0),
        ("AEB_ALARM", "LKAS_HUD", 0),
        ("SET_SPEED", "ACC_CMD_HUD", 0),
        ("FOLLOW_DISTANCE", "ACC_CMD_HUD", 0),
        ("GAS_PEDAL_STEP", "GAS_PEDAL_2", 0),
        ("UI_SPEED", "BUTTONS", 0),
        ("LKC_BTN", "BUTTONS", 0),
        ("DISTANCE_BTN", "BUTTONS", 0),
        ("CRUISE_STANDSTILL", "ACC_BRAKE", 0),
        ("MAGNITUDE", "ACC_BRAKE", 0),
        ("AEB_1019", "ACC_BRAKE", 0)
      ]
    else:
      signals += [
        ("MAIN_TORQUE", "STEERING_TORQUE", 0),
        ("STEER_ANGLE", "STEERING_ANGLE_SENSOR", 0.),
        ("AEB_ALARM", "ADAS_HUD", 0),
        ("BRAKE_REQ", "ADAS_AEB", 0),
        ("WHEELSPEED_B", "WHEEL_SPEED", 0.)
      ]

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, [], 0)
