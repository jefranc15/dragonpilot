"""V7.1-R1 Panda source-to-firmware binding and policy audit.

ALL TESTS ARE OFFLINE: a passing result verifies that the audited weakness
is still present and the documented ownership maps are internally consistent;
it does NOT approve road use.
"""
import pathlib
import re
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[4]
SAFETY = (ROOT / "panda/board/safety.h").read_text()
HOOK = (ROOT / "panda/board/safety/safety_dnga_stock.h").read_text()
LEGACY = (ROOT / "panda/board/safety/safety_toyota.h").read_text()
DECLS = (ROOT / "panda/board/safety_declarations.h").read_text()
CAPNP = (ROOT / "cereal/car.capnp").read_text()
IFACE = (ROOT / "selfdrive/car/dnga/interface.py").read_text()
CTRL = (ROOT / "selfdrive/car/dnga/carcontroller.py").read_text()
BOARDD = (ROOT / "selfdrive/boardd/boardd.cc").read_text()
BOARD_MAIN = (ROOT / "panda/board/main.c").read_text()


class PandaBindingTests(unittest.TestCase):
  def test_enum_and_registry_is_model_28(self):
    self.assertIn("dngaStock @28;", CAPNP)
    self.assertRegex(SAFETY, r"#define\s+SAFETY_DNGA_STOCK\s+28U")
    self.assertIn("{SAFETY_DNGA_STOCK, &dnga_stock_hooks}", SAFETY)
    self.assertIn("CarParams.SafetyModel.dngaStock", IFACE)

  def test_boardd_uses_cereal_safety_config(self):
    self.assertIn("panda->set_safety_model(safety_model, safety_param);", BOARDD)
    self.assertIn("ps.setSafetyModel(cereal::CarParams::SafetyModel(", BOARDD)
    self.assertIn("getUnsafeMode()", BOARDD)

  def test_no_op_longitudinal_on_current_branch(self):
    self.assertIn("ret.openpilotLongitudinalControl = False", IFACE)
    self.assertIn("create_can_steer_command(", CTRL)
    self.assertNotIn("dnga_create_brake_command", CTRL)
    self.assertNotIn("dnga_create_accel_command", CTRL)

  def test_safety_has_relay_malfunction_wrapper(self):
    self.assertRegex(SAFETY, r"int safety_tx_hook\(CANPacket_t \*to_send\)")
    self.assertIn("relay_malfunction ? -1 : current_hooks->tx(to_send)", SAFETY)
    self.assertIn("relay_malfunction ? -1 : current_hooks->fwd(bus_num, to_fwd)", SAFETY)

  def test_header_still_fails_vehicle_specific_safety(self):
    # A GREEN result means the known road-readiness blocker is still present.
    self.assertIn("return 1;", HOOK)  # RX does not check signals
    self.assertIn("controls_allowed = true;", HOOK)
    self.assertIn("return &dnga_rx_checks;", HOOK)
    self.assertRegex(LEGACY, r"\.len\s*=\s*0\s*,")
    self.assertIn("return GET_ADDR(to_send) == 0x1D0 && GET_BUS(to_send) == 0", HOOK)
    self.assertNotIn("driver_limit_check(", HOOK)
    self.assertNotIn("rt_rate_limit_check(", HOOK)
    self.assertNotIn("addr_safety_check(", HOOK)
    self.assertNotIn("generic_rx_checks(", HOOK)

  def test_panda_has_heartbeat_fallback_but_not_sufficient(self):
    self.assertIn("HEARTBEAT_IGNITION_CNT_ON 5U", BOARD_MAIN)
    self.assertIn("set_safety_mode(SAFETY_SILENT, 0U);", BOARD_MAIN)
    # Generic RX protections exist in the framework, but the DNGA stock
    # hook never calls them.
    self.assertIn("void generic_rx_checks(", SAFETY)
    self.assertNotIn("generic_rx_checks(", HOOK)


if __name__ == "__main__":
  unittest.main()
