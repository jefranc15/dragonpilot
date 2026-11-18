// DNGA HEV V7 stock-longitudinal prototype.
//
// Factory camera owns 0x271 (brake), 0x273 (ACC target/mode), and 0x274 (HUD).
// Openpilot may transmit only 0x1D0 steering. No experimental CAN filtering,
// braking strength modifications, or synthetic leads are permitted.
//
// WARNING: bench/HIL validation required for the vehicle-specific safety
// envelope, steering torque limits and OEM camera forwarding. This safety
// hook deliberately cannot transmit longitudinal requests.

static int dnga_stock_rx_hook(CANPacket_t *to_push) {
  UNUSED(to_push);
  // Retains existing DNGA lateral gating behavior for offline development.
  // Production use requires proper tested RX checks and controls_allowed
  // conditions; do not treat this as road-qualified.
  return 1;
}

static int dnga_stock_tx_hook(CANPacket_t *to_send) {
  // A distinct Panda safety ID is used so an old V6 Panda cannot silently
  // accept V7 stocklong mode. Deny ACC and HUD replacements completely.
  return GET_ADDR(to_send) == 0x1D0 && GET_BUS(to_send) == 0;
}

static int dnga_stock_fwd_hook(int bus_num, CANPacket_t *to_fwd) {
  if (bus_num == 0) {
    return 2;  // Powertrain to camera.
  }
  if (bus_num == 2) {
    // Only replace OEM steering 0x1D0; forward every OEM ACC message
    // including brake 0x271, target 0x273 and HUD 0x274 unchanged.
    return GET_ADDR(to_fwd) == 0x1D0 ? -1 : 0;
  }
  return -1;
}

static const addr_checks *dnga_stock_init(int16_t param) {
  UNUSED(param);
  controls_allowed = true;  // Pre-existing DNGA steering latch model.
  return &dnga_rx_checks;
}

const safety_hooks dnga_stock_hooks = {
  .init = dnga_stock_init,
  .rx = dnga_stock_rx_hook,
  .tx = dnga_stock_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = dnga_stock_fwd_hook,
};
