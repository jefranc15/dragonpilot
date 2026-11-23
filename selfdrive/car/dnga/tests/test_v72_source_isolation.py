"""V7.2 model 29 is bench-locked and cannot replace active V7.1 model 28."""
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[4]
SAFETY = (ROOT / "panda/board/safety.h").read_text()
V72 = (ROOT / "panda/board/safety/safety_dnga_v72.h").read_text()
V70 = (ROOT / "panda/board/safety/safety_dnga_stock.h").read_text()
IFACE = (ROOT / "selfdrive/car/dnga/interface.py").read_text()
CTRL = (ROOT / "selfdrive/car/dnga/carcontroller.py").read_text()
CAR = (ROOT / "cereal/car.capnp").read_text()


class V72SourceIsolationTests(unittest.TestCase):
  def test_distinct_enum_ids(self):
    self.assertIn("dngaStock @28;", CAR)
    self.assertIn("dngaV72 @29;", CAR)
    self.assertIn("#define SAFETY_DNGA_V72 29U", SAFETY)
    self.assertIn("{SAFETY_DNGA_V72, &dnga_v72_hooks}", SAFETY)
    self.assertIn('{SAFETY_DNGA_STOCK, &dnga_stock_hooks}', SAFETY)

  def test_v72_not_selected_by_running_interface(self):
    self.assertIn("CarParams.SafetyModel.dngaStock", IFACE)
    self.assertNotIn("CarParams.SafetyModel.dngaV72", IFACE)
    self.assertIn("ret.openpilotLongitudinalControl = False", IFACE)
    self.assertIn("ret.steerRatio = 17.0", IFACE)
    self.assertIn("SafetyModel.dngaStock", CTRL)

  def test_v72_tx_unconditionally_locked(self):
    body = V72.split("static int dnga_v72_tx_hook(", 1)[1].split(
      "static int dnga_v72_fwd_hook(", 1)[0]
    self.assertRegex(body, r"return\s+0\s*;")
    self.assertNotIn("return 1", body)
    self.assertNotIn("#ifdef", body)
    self.assertNotIn("SAFETY_PARAM", body)

  def test_v72_forwards_camera_steering_while_locked(self):
    body = V72.split("static int dnga_v72_fwd_hook(", 1)[1].split(
      "static const addr_checks *dnga_v72_init(", 1)[0]
    self.assertRegex(body, r"if \(bus_num == 2\)[\s\S]*?return 0;")
    self.assertNotIn("GET_ADDR(m)", body)

  def test_unmodified_v7_ownership(self):
    self.assertIn("return GET_ADDR(to_send) == 0x1D0 && GET_BUS(to_send) == 0", V70)
    self.assertIn("create_can_steer_command(", CTRL)
    for banned in ("dnga_create_brake_command", "dnga_create_accel_command",
                   "dnga_create_hud"):
      self.assertNotIn(banned, CTRL)

  def test_v72_provisional_not_road_certified(self):
    self.assertIn("BENCH-ONLY", V72)
    self.assertIn("controls_allowed = false;", V72)
    self.assertIn("dnga_v72_steer_checksum_ok", V72)
    self.assertIn("dnga_v72_steer_candidate_ok", V72)
    self.assertIn("dnga_v72_driver_torque", V72)
    self.assertIn("dnga_v72_cam_engaged", V72)


if __name__ == "__main__":
  unittest.main()
