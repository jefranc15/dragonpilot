from common.numpy_fast import clip
from cereal import car
from selfdrive.config import Conversions as CV

class SetDistance:
  aggresive = 0
  normal = 1
  far = 2

def compute_set_distance(state):
  if state == SetDistance.aggresive:
    return 2
  elif state == SetDistance.normal:
    return 1
  else:
    return 0

def lkc_checksum(addr,dat):
  return ( addr + len(dat) + 1 + 1 + sum(dat)) & 0xFF

def dnga_checksum(addr,dat):
  return ( addr + len(dat) + 1 + 2 + sum(dat)) & 0xFF

def create_can_steer_command(packer, steer, steer_req, raw_cnt):
  values = {
    "STEER_REQ": steer_req,
    "STEER_CMD": -steer if steer_req else 0,
    "COUNTER": raw_cnt,
    "SET_ME_1": 1,
    "SET_ME_1_2": 1,
  }
  dat = packer.make_can_msg("STEERING_LKAS", 0, values)[2]
  crc = lkc_checksum(0x1d0, dat[:-1])
  values["CHECKSUM"] = crc
  return packer.make_can_msg("STEERING_LKAS", 0, values)

def dnga_aeb_warning(packer):
  values = {"AEB_ALARM": 1}
  dat = packer.make_can_msg("ADAS_HUD", 0, values)[2]
  checksum = dnga_checksum(681, dat[:-1])
  values["CHECKSUM"] = checksum
  return packer.make_can_msg("ADAS_HUD", 0, values)

def dnga_create_brake_command(packer, brake_state, pump_reaction, brake_mag, idx):
  values = {
    "COUNTER": idx,
    "BRAKE_STATE": brake_state,
    "UNKNOWN_BYTE_2": 0x00,
    "PUMP_REACTION2": pump_reaction,
    "MAGNITUDE": brake_mag,
  }

  dat = packer.make_can_msg("ACC_BRAKE", 0, values)[2]
  crc = dnga_checksum(0x271, dat[:-1])
  values["CHECKSUM"] = crc

  return packer.make_can_msg("ACC_BRAKE", 0, values)

def dnga_create_accel_command(packer, set_speed, acc_rdy, enabled, is_lead, des_speed, brake_amt, set_distance):
  is_braking = brake_amt > 0.01

  values = {
    "SET_SPEED": set_speed * CV.MS_TO_KPH,
    "FOLLOW_DISTANCE": compute_set_distance(set_distance),
    "IS_LEAD": is_lead,
    "IS_ACCEL": (not is_braking) and enabled,
    "IS_DECEL": is_braking and enabled,
    "SET_ME_1_2": acc_rdy, 
    "SET_ME_1": 1,
    "SET_0_WHEN_ENGAGE": not enabled,
    "SET_1_WHEN_ENGAGE": enabled,
    "ACC_CMD": des_speed * CV.MS_TO_KPH if enabled else 0,
  }

  dat = packer.make_can_msg("ACC_CMD_HUD", 0, values)[2]
  crc = (dnga_checksum(0x273, dat[:-1]))
  values["CHECKSUM"] = crc
  return packer.make_can_msg("ACC_CMD_HUD", 0, values)

def dnga_create_hud(packer, lkas_rdy, enabled, llane_visible, rlane_visible, ldw, fcw, aeb, front_depart, ldp_off, fcw_off):
  values = {
    "LKAS_SET": lkas_rdy,
    "LKAS_ENGAGED": enabled,
    "LDA_ALERT": ldw,
    "LDA_OFF": ldp_off,
    "LANE_RIGHT_DETECT": rlane_visible,
    "LANE_LEFT_DETECT": llane_visible,
    "SET_ME_X02": 0x2,
    "AEB_ALARM": fcw,
    "AEB_BRAKE": aeb,
    "FRONT_DEPART": front_depart,
    "FCW_DISABLE": fcw_off,
  }

  dat = packer.make_can_msg("LKAS_HUD", 0, values)[2]
  crc = (dnga_checksum(0x274, dat[:-1]))
  values["CHECKSUM"] = crc
  return packer.make_can_msg("LKAS_HUD", 0, values)
