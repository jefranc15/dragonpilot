// DNGA HEV V7.2: BENCH-ONLY safety prototype, NOT an actuation release.
//
// Design intent: replace the permissive V7.0 dngaStock hook with a separately
// registered, fail-closed prototype. The requested "V7.2" branch does NOT
// select this mode in CarInterface, and TX is unconditionally denied below.
//
// Reproducing Toyota camera ACC 0x271 / 0x273 / 0x274 unchanged is essential.
// While actuation is locked, *factory steering* 0x1D0 is also forwarded.
// The proto must not block camera LKAS without a permitted replacement.
//
// WARNING: torque limits and DBC signal layouts below are preliminary
// audit hypotheses, NOT vehicle-validated safety limits. Do not change
// dnga_v72_tx_hook to enable output without independent HIL qualification.

#define DNGA_V72_STEER_ADDR 0x1D0U
#define DNGA_V72_ACC_ADDR 0x273U
#define DNGA_V72_MAX_STEER 255
#define DNGA_V72_DELTA_UP 20
#define DNGA_V72_DELTA_DOWN 30
#define DNGA_V72_MAX_DRIVER_TORQUE 20
#define DNGA_V72_CAMERA_TIMEOUT_US 150000U
#define DNGA_V72_PEDAL_TIMEOUT_US 100000U
#define DNGA_V72_DRIVER_TIMEOUT_US 100000U

static AddrCheckStruct dnga_v72_addr_checks[] = {
  {
    // Camera ACC: 8-byte 0x273 at 20 Hz, additive checksum, no established
    // counter. Firmware safety_tick also detects lengthy camera silence,
    // but the policy below uses a stricter microsecond freshness window.
    .msg = {{0x273, 2, 8, true, 0, 50000}, {0}},
  },
};
static const addr_checks dnga_v72_rx_checks = {
  .check = dnga_v72_addr_checks,
  .len = sizeof(dnga_v72_addr_checks) / sizeof(dnga_v72_addr_checks[0]),
};

static bool dnga_v72_cam_known = false;
static bool dnga_v72_cam_engaged = false;
static bool dnga_v72_brake_known = false;
static bool dnga_v72_gas_known = false;
static bool dnga_v72_driver_known = false;
static bool dnga_v72_brake = false;
static bool dnga_v72_gas = false;
static int dnga_v72_driver_torque = 0;
static uint32_t dnga_v72_cam_ts = 0U;
static uint32_t dnga_v72_brake_ts = 0U;
static uint32_t dnga_v72_gas_ts = 0U;
static uint32_t dnga_v72_driver_ts = 0U;
static int dnga_v72_last_torque = 0;
static int dnga_v72_last_counter = -1;

static uint8_t dnga_v72_checksum(CANPacket_t *m) {
  return GET_BYTE(m, 7);
}

static uint8_t dnga_v72_acc_checksum(CANPacket_t *m) {
  // Original 0x273 encoder: addr + 7 payload bytes + 1 + 2.
  int sum = (int)GET_ADDR(m) + 7 + 1 + 2;
  for (int i = 0; i < 7; i++) {
    sum += (int)GET_BYTE(m, i);
  }
  return (uint8_t)(sum & 0xFF);
}

static bool dnga_v72_steer_checksum_ok(CANPacket_t *m) {
  if (GET_LEN(m) != 8) {
    return false;
  }
  // Original 0x1D0 encoder: addr + 7 payload bytes + 1 + 1.
  int sum = (int)DNGA_V72_STEER_ADDR + 7 + 1 + 1;
  for (int i = 0; i < 7; i++) {
    sum += (int)GET_BYTE(m, i);
  }
  return (uint8_t)(sum & 0xFF) == GET_BYTE(m, 7);
}

static int dnga_v72_decode_steer(CANPacket_t *m) {
  // DBC 0x1D0 STEER_CMD : 7|11@0-, signed Motorola:
  // byte0 [7:0], byte1 [7:5].
  int raw = ((int)GET_BYTE(m, 0) << 3) | ((int)GET_BYTE(m, 1) >> 5);
  return (raw & 0x400) ? raw - 0x800 : raw;
}

static int dnga_v72_decode_driver(CANPacket_t *m) {
  // DBC 0x164 MAIN_TORQUE : 9|12@0-, pending raw-log verification.
  int raw = (((int)GET_BYTE(m, 1) & 0x3) << 10) |
            ((int)GET_BYTE(m, 2) << 2) |
            ((int)GET_BYTE(m, 3) >> 6);
  return (raw & 0x800) ? raw - 0x1000 : raw;
}

static void dnga_v72_disengage(void) {
  controls_allowed = false;
  dnga_v72_last_torque = 0;
  dnga_v72_last_counter = -1;
}

static int dnga_v72_rx_hook(CANPacket_t *m) {
  const uint32_t now = microsecond_timer_get();
  const int bus = GET_BUS(m);
  const int addr = GET_ADDR(m);
  const int len = GET_LEN(m);

  if ((bus == 2) && (addr == DNGA_V72_ACC_ADDR)) {
    const bool valid = (len == 8) && addr_safety_check(m, &dnga_v72_rx_checks,
                                                       dnga_v72_checksum,
                                                       dnga_v72_acc_checksum, NULL);
    if (!valid) {
      dnga_v72_cam_known = false;
      dnga_v72_cam_engaged = false;
      dnga_v72_disengage();
      return 0;
    }
    dnga_v72_cam_known = true;
    dnga_v72_cam_ts = now;
    dnga_v72_cam_engaged = (GET_BYTE(m, 1) & 0x20U) != 0U;
    if (!dnga_v72_cam_engaged) {
      dnga_v72_disengage();
    }
  } else if ((bus == 0) && (addr == 0xA1U)) {
    // Brake status from DBC 0x0A1, BRAKE_ENGAGED bit 5.
    if (len != 8) {
      dnga_v72_brake_known = false;
      dnga_v72_disengage();
      return 0;
    }
    dnga_v72_brake_known = true;
    dnga_v72_brake_ts = now;
    dnga_v72_brake = (GET_BYTE(m, 0) & 0x20U) != 0U;
    if (dnga_v72_brake) {
      dnga_v72_disengage();
    }
  } else if ((bus == 1) && (addr == 0x277U)) {
    // Raw pedal on bus1 observed by CarInterface. Not a calibrated
    // independent gas safety source until matching records are verified.
    if (len != 8) {
      dnga_v72_gas_known = false;
      dnga_v72_disengage();
      return 0;
    }
    uint32_t raw = ((uint32_t)GET_BYTE(m, 1) << 8) | (uint32_t)GET_BYTE(m, 2);
    dnga_v72_gas_known = true;
    dnga_v72_gas_ts = now;
    dnga_v72_gas = raw > 100U;
    if (dnga_v72_gas) {
      dnga_v72_disengage();
    }
  } else if ((bus == 0) && (addr == 0xA4U)) {
    if (len != 8) {
      dnga_v72_driver_known = false;
      dnga_v72_disengage();
      return 0;
    }
    dnga_v72_driver_known = true;
    dnga_v72_driver_ts = now;
    dnga_v72_driver_torque = dnga_v72_decode_driver(m);
    if ((dnga_v72_driver_torque > DNGA_V72_MAX_DRIVER_TORQUE) ||
        (dnga_v72_driver_torque < -DNGA_V72_MAX_DRIVER_TORQUE)) {
      dnga_v72_disengage();
    }
  }

  // Deliberately NO controls_allowed=true here. Camera ACC is a REQUEST,
  // not an authenticated ECU engagement ACK. Even complete/fresh inputs
  // cannot independently authorize steering in this prototype.
  return 1;
}

static bool dnga_v72_steer_candidate_ok(CANPacket_t *m) {
  if ((GET_ADDR(m) != DNGA_V72_STEER_ADDR) ||
      (GET_BUS(m) != 0) || (GET_LEN(m) != 8) ||
      !dnga_v72_steer_checksum_ok(m)) {
    return false;
  }
  // DBC: SET_ME_1 bit22 -> byte2 bit6; SET_ME_1_2 bit30 -> byte3 bit6.
  if (((GET_BYTE(m, 2) & 0x40U) == 0U) ||
      ((GET_BYTE(m, 3) & 0x40U) == 0U)) {
    return false;
  }
  const bool steer_requested = (GET_BYTE(m, 2) & 0x20U) != 0U;
  const int torque = dnga_v72_decode_steer(m);
  const int counter = GET_BYTE(m, 1) & 0x0FU;
  if ((torque > DNGA_V72_MAX_STEER) || (torque < -DNGA_V72_MAX_STEER)) {
    return false;
  }
  if (!steer_requested && torque != 0) {
    return false;
  }
  if ((dnga_v72_last_counter >= 0) &&
      (counter != ((dnga_v72_last_counter + 1) & 0x0F))) {
    return false;
  }

  if (steer_requested) {
    const uint32_t now = microsecond_timer_get();
    const bool all_fresh = dnga_v72_cam_known && dnga_v72_brake_known &&
      dnga_v72_gas_known && dnga_v72_driver_known &&
      ((uint32_t)(now - dnga_v72_cam_ts) <= DNGA_V72_CAMERA_TIMEOUT_US) &&
      ((uint32_t)(now - dnga_v72_brake_ts) <= DNGA_V72_PEDAL_TIMEOUT_US) &&
      ((uint32_t)(now - dnga_v72_gas_ts) <= DNGA_V72_PEDAL_TIMEOUT_US) &&
      ((uint32_t)(now - dnga_v72_driver_ts) <= DNGA_V72_DRIVER_TIMEOUT_US);
    if (!controls_allowed || !dnga_v72_cam_engaged || !all_fresh ||
        dnga_v72_brake || dnga_v72_gas ||
        dnga_v72_driver_torque > DNGA_V72_MAX_DRIVER_TORQUE ||
        dnga_v72_driver_torque < -DNGA_V72_MAX_DRIVER_TORQUE) {
      return false;
    }

    const int delta = torque - dnga_v72_last_torque;
    const bool increasing = (torque > 0 && delta > 0) || (torque < 0 && delta < 0);
    const int limit = increasing ? DNGA_V72_DELTA_UP : DNGA_V72_DELTA_DOWN;
    if (delta > limit || delta < -limit) {
      return false;
    }
  }

  dnga_v72_last_counter = counter;
  dnga_v72_last_torque = torque;
  return true;
}

static int dnga_v72_tx_hook(CANPacket_t *m) {
  // Evaluates a safety candidate for native tests and bench diagnostics,
  // but ALWAYS DENIES HOST ACTUATION regardless of state. This is
  // intentionally not controlled by any firmware flag or safetyParam.
  UNUSED(m);
  return 0;
}

static int dnga_v72_fwd_hook(int bus_num, CANPacket_t *m) {
  UNUSED(m);
  if (bus_num == 0) {
    return 2;
  }
  if (bus_num == 2) {
    // In TX-locked bench mode, don't suppress the OEM camera LKAS.
    // Otherwise we would remove the only steering source!
    return 0;
  }
  return -1;
}

static const addr_checks *dnga_v72_init(int16_t param) {
  UNUSED(param);
  controls_allowed = false;
  dnga_v72_cam_known = false;
  dnga_v72_cam_engaged = false;
  dnga_v72_brake_known = false;
  dnga_v72_gas_known = false;
  dnga_v72_driver_known = false;
  dnga_v72_brake = false;
  dnga_v72_gas = false;
  dnga_v72_driver_torque = 0;
  dnga_v72_cam_ts = 0U;
  dnga_v72_brake_ts = 0U;
  dnga_v72_gas_ts = 0U;
  dnga_v72_driver_ts = 0U;
  dnga_v72_last_torque = 0;
  dnga_v72_last_counter = -1;
  return &dnga_v72_rx_checks;
}

const safety_hooks dnga_v72_hooks = {
  .init = dnga_v72_init,
  .rx = dnga_v72_rx_hook,
  .tx = dnga_v72_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = dnga_v72_fwd_hook,
};
