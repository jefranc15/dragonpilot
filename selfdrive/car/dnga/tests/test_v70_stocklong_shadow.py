"""V7 stock-longitudinal shadow and CAN ownership regression (offline only)."""
import pathlib
import sys
import unittest

TEST_DIR = pathlib.Path(__file__).resolve().parent
DNGA = TEST_DIR.parent
REPO = DNGA.parents[2]
sys.path.insert(0, str(DNGA))

from stock_long_shadow import StockBrakeShadow


class ShadowTests(unittest.TestCase):
  def test_gentle_brake_shadow_ramps_without_actuating(self):
    s = StockBrakeShadow()
    x = s.observe(100, 100, 0x21, 180)
    self.assertTrue(x.valid)
    self.assertAlmostEqual(x.raw_strength, .20)
    self.assertAlmostEqual(x.proposed_strength, .025)
    self.assertEqual(x.reason, "ordinary_brake_shadow_slew")
    self.assertEqual(x.proposed_magnitude, 198)

  def test_repeated_samples_ramp_diagnostic_only(self):
    s = StockBrakeShadow()
    a = s.observe(5, 5, 0x21, 180)
    b = s.observe(10, 10, 0x21, 180)
    self.assertGreater(b.proposed_strength, a.proposed_strength)

  def test_strong_brake_is_proposed_unchanged(self):
    x = StockBrakeShadow().observe(5, 5, 0x21, 140)
    self.assertTrue(x.valid)
    self.assertAlmostEqual(x.raw_strength, x.proposed_strength)
    self.assertEqual(x.reason, "strong_brake_unmodified")

  def test_all_active_states(self):
    for state in (0x21, 0x30, 0x31):
      self.assertTrue(StockBrakeShadow().observe(5, 5, state, 185).valid)

  def test_release_resets_immediately(self):
    s = StockBrakeShadow()
    s.observe(5, 5, 0x21, 130)
    x = s.observe(10, 10, 0x01, 200)
    self.assertEqual(x.proposed_strength, 0)
    self.assertEqual(x.raw_magnitude, x.proposed_magnitude)

  def test_stale_camera_fails_closed(self):
    s = StockBrakeShadow()
    x = s.observe(100, 1, 0x21, 180)
    self.assertFalse(x.valid)
    self.assertEqual(x.proposed_strength, 0)

  def test_future_camera_timestamp_rejected(self):
    self.assertFalse(StockBrakeShadow().observe(100, 105, 0x21, 180).valid)

  def test_unknown_brake_state_rejected(self):
    self.assertFalse(StockBrakeShadow().observe(100, 100, 0x99, 180).valid)

  def test_bad_magnitude_rejected(self):
    self.assertFalse(StockBrakeShadow().observe(100, 100, 0x21, 255).valid)

  def test_duplicate_sample_rejected(self):
    s = StockBrakeShadow()
    self.assertTrue(s.observe(10, 10, 0x21, 190).valid)
    self.assertFalse(s.observe(10, 10, 0x21, 190).valid)


class SingleOwnerSourceContractTests(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.controller = (DNGA / "carcontroller.py").read_text()
    cls.interface = (DNGA / "interface.py").read_text()
    cls.safety = (REPO / "panda/board/safety/safety_dnga_stock.h").read_text()
    cls.registry = (REPO / "panda/board/safety.h").read_text()
    cls.capnp = (REPO / "cereal/car.capnp").read_text()

  def test_no_op_longitudinal_send_path(self):
    for prohibited in ("dnga_create_brake_command", "dnga_create_accel_command", "LongitudinalController", "dnga_create_hud"):
      self.assertNotIn(prohibited, self.controller)

  def test_interface_factory_longitudinal(self):
    self.assertIn("ret.openpilotLongitudinalControl = False", self.interface)
    self.assertIn("CarParams.SafetyModel.dngaStock", self.interface)

  def test_safety_unique_id_consistent(self):
    self.assertIn("dngaStock @28;", self.capnp)
    self.assertIn("#define SAFETY_DNGA_STOCK 28U", self.registry)
    self.assertIn("{SAFETY_DNGA_STOCK, &dnga_stock_hooks}", self.registry)

  def test_panda_only_op_steering_tx(self):
    self.assertIn("GET_ADDR(to_send) == 0x1D0", self.safety)
    self.assertIn("GET_BUS(to_send) == 0", self.safety)
    self.assertIn("GET_ADDR(to_fwd) == 0x1D0 ? -1 : 0", self.safety)

  def test_filter_does_not_create_can(self):
    shadow_text = (DNGA / "stock_long_shadow.py").read_text()
    self.assertNotIn("make_can_msg(", shadow_text)
    self.assertNotIn("packer", shadow_text)
    self.assertNotIn("sendcan", shadow_text)


if __name__ == "__main__":
  unittest.main()
