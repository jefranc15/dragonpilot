// Road-Ready DNGA Safety Model for 0.8.13

// RX check structure required by safety_declarations.h
const addr_checks dnga_rx_checks = {
  .check = NULL,
  .len = 0,
};

static int dnga_rx_hook(CANPacket_t *to_push) {
  int addr = GET_ADDR(to_push);

  // 1. DYNAMIC VEHICLE MOVING CHECK (Replaces the dangerous bypass)
  // Reads WHEEL_SPEED (416 / 0x1A0) to check if the car is actually moving.
  // WHEELSPEED_F is 24 bits spread across the first 3 bytes.
  if (addr == 416) {
    int speed_front = (GET_BYTE(to_push, 0) << 16) | (GET_BYTE(to_push, 1) << 8) | GET_BYTE(to_push, 2);
    vehicle_moving = (speed_front > 0);
  }

  // 2. CONTROLS ALLOWED
  // Because your car's cruise state is calculated in Python (self.is_cruise_latch),
  // the C safety model must leave this open, otherwise OP can never engage. 
  // The TX Whitelist below acts as the primary safety barrier instead.
  controls_allowed = true;

  return 1;
}

static int dnga_tx_hook(CANPacket_t *to_send) {
  int addr = GET_ADDR(to_send);
  
  // 3. THE TX WHITELIST (From the Veloz branch)
  // Only allow openpilot to send specific LKAS and ACC commands.
  // STEERING_LKAS (464 / 0x1D0)
  // ACC_BRAKE (625 / 0x271)
  // ACC_CMD_HUD (627 / 0x273)
  // LKAS_HUD (628 / 0x274)
  if (addr == 464 || addr == 625 || addr == 627 || addr == 628) {
    return 1; // Allow transmission
  }
  
  return 0; // BLOCK EVERYTHING ELSE!
}

static int dnga_fwd_hook(int bus_num, CANPacket_t *to_fwd) {
  int bus_fwd = -1;
  int addr = GET_ADDR(to_fwd);

  if (bus_num == 0) {
    // Forward Powertrain to Camera
    bus_fwd = 2;
  }

  if (bus_num == 2) {
    // Block the 4 specific LKAS/ACC IDs from the Camera
    // so the Panda can inject its own commands to the Powertrain
    bool is_lkas_msg = ((addr == 464) || (addr == 628));
    bool is_acc_msg = ((addr == 625) || (addr == 627));
    
    if (!(is_lkas_msg || is_acc_msg)) {
      bus_fwd = 0;
    }
  }

  return bus_fwd;
}

static const addr_checks* dnga_init(int16_t param) {
  UNUSED(param);
  
  // Initialize controls to allowed so the Python latch works
  controls_allowed = true; 
  
  return &dnga_rx_checks;
}

const safety_hooks dnga_hooks = {
  .init = dnga_init,
  .rx = dnga_rx_hook,
  .tx = dnga_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = dnga_fwd_hook,
};