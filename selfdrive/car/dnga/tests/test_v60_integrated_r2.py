"""v6.0 isolated R2 checks without native cereal/DBC or vehicle CAN access."""
from dataclasses import dataclass
from importlib import util
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import patch
import ast
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
CONTROLLER = ROOT / "carcontroller.py"
LONGITUDINAL = ROOT / "longitudinal.py"


class FakePlanReader:
  def __init__(self, *args, **kwargs):
    self.updated = {"longitudinalPlan": True}
    self.msg = SimpleNamespace(
      longitudinalPlanSource="turn", hasLead=False,
      accels=[-0.20, -0.20, -0.30] + [-0.20] * 10,
    )

  def update(self, timeout):
    return None

  def __getitem__(self, key):
    return self.msg


class FakeLegacyLongitudinal:
  instances = []
  def __init__(self):
    self.calls = 0
    self.__class__.instances.append(self)

  def update(self, enabled, CS, frame, accel, cancel, lead):
    self.calls += 1
    return Cmd(enabled=True, brake_state=0x30, magnitude=1224, pump=-0.4)


@dataclass
class Cmd:
  enabled: bool = True
  lead: bool = False
  speed: float = 20.0
  is_accel: bool = False
  is_decel: bool = False
  brake_state: int = 0x01
  pump: float = 0.0
  magnitude: int = 200


class TestV60IntegratedR2(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    stubs = {}
    for name in ("selfdrive", "selfdrive.car", "selfdrive.car.dnga",
                 "cereal", "selfdrive.car.dnga.longitudinal_v50_legacy",
                 "selfdrive.car.dnga.longitudinal_kinematic_guard",
                 "selfdrive.car.dnga.dnga_hybrid_feedback",
                 "selfdrive.car.dnga.values"):
      stub = ModuleType(name)
      stub.__path__ = []
      stubs[name] = stub

    messaging = ModuleType("cereal.messaging")
    messaging.SubMaster = FakePlanReader
    stubs["cereal.messaging"] = messaging
    stubs["cereal"].messaging = messaging

    stubs["selfdrive.car.dnga.longitudinal_v50_legacy"].LongitudinalCommand = Cmd
    stubs["selfdrive.car.dnga.longitudinal_kinematic_guard"].LongitudinalController = FakeLegacyLongitudinal
    stubs["selfdrive.car.dnga.dnga_hybrid_feedback"].hybrid_feedback_snapshot = lambda CS, f: {"fresh": True}
    stubs["selfdrive.car.dnga.dnga_hybrid_feedback"].hybrid_brake_feedback_state = lambda f: (True, False)
    stubs["selfdrive.car.dnga.values"].BrakeState = SimpleNamespace(
      DISABLED=0x00, READY=0x01, BRAKING=0x21, CREEPING=0x31, HOLD=0x30)
    stubs["selfdrive.car.dnga.values"].CarControllerParams = SimpleNamespace(ACC_STEP=5)
    with patch.dict(sys.modules, stubs):
      spec = util.spec_from_file_location("v60_r2_test_module", LONGITUDINAL)
      module = util.module_from_spec(spec)
      sys.modules[spec.name] = module
      try:
        spec.loader.exec_module(module)
      finally:
        sys.modules.pop(spec.name, None)
    cls.module = module

  def setUp(self):
    self.ctrl = self.module.LongitudinalController()
    self.cs = SimpleNamespace(
      out=SimpleNamespace(vEgo=20.0, aEgo=-0.1,
                          brakePressed=False, gasPressed=False,
                          cruiseState=SimpleNamespace(enabled=True)),
      hybrid_feedback_rx_frame_275=5,
      hybrid_feedback_rx_frame_08c=5,
      hybrid_brake_request_raw_275=-163,
      hybrid_friction_raw_08c=0,
    )

  def test_direct_1216_from_first_decel_no_base_coast(self):
    out = self.ctrl.update(True, self.cs, 5, -0.20, False, False)
    self.assertEqual((out.brake_state, out.magnitude, out.pump),
                     (0x21, 1216, -0.4))
    self.assertTrue(out.is_decel)
    self.assertFalse(out.is_accel)
    self.assertLess(out.speed, self.cs.out.vEgo)
    self.assertEqual(self.ctrl.low_speed.calls, 0)

  def test_slew_and_release_without_shadow(self):
    self.ctrl.update(True, self.cs, 5, -0.2, False, False)
    self.cs.hybrid_feedback_rx_frame_275 = 10
    self.cs.hybrid_feedback_rx_frame_08c = 10
    next_cmd = self.ctrl.update(True, self.cs, 10, -0.2, False, False)
    self.assertGreaterEqual(next_cmd.magnitude, 1190)
    self.ctrl.plan_sm.msg = SimpleNamespace(
      longitudinalPlanSource="cruise", hasLead=False,
      accels=[0.1] * 13)
    self.cs.hybrid_feedback_rx_frame_275 = 15
    self.cs.hybrid_feedback_rx_frame_08c = 15
    release = self.ctrl.update(True, self.cs, 15, 0.1, False, False)
    self.assertEqual((release.brake_state, release.magnitude),
                     (0x01, 1224))
    self.assertFalse(release.is_accel)

  def test_low_speed_compatibility_adapter(self):
    self.cs.out.vEgo = 3.0
    out = self.ctrl.update(True, self.cs, 5, -0.2, False, True)
    self.assertEqual(out.brake_state, 0x30)
    self.assertEqual(self.ctrl.low_speed.calls, 1)

  def test_plan_missing_never_generates_positive_accel(self):
    self.ctrl.plan_sm.updated["longitudinalPlan"] = False
    out = self.ctrl.update(True, self.cs, 100, 1.0, False, False)
    self.assertEqual((out.brake_state, out.is_accel), (0x01, False))
    self.assertEqual(self.ctrl.last_authority, "stale_plan")

  def test_cancel_reset(self):
    out = self.ctrl.update(True, self.cs, 5, -0.2, True, False)
    self.assertFalse(out.is_accel)
    self.assertEqual(out.brake_state, 0x01)
    self.assertEqual(self.ctrl.last_authority, "inactive")

  def test_no_observer_or_r2_override_in_transmitter(self):
    code = CONTROLLER.read_text()
    self.assertNotIn("r2_v4_shadow", code)
    self.assertNotIn("r2_v4_actuator", code)
    self.assertNotIn("r2_v4_direct_authority", code)
    self.assertEqual(code.count("self.longitudinal.update("), 1)
    self.assertIn("dnga_create_brake_command(", code)
    self.assertIn("dnga_create_accel_command(", code)
    tree = ast.parse(code)
    opts = {n.targets[0].id: n.value.value
            for n in tree.body if isinstance(n, ast.Assign)
            and len(n.targets) == 1 and isinstance(n.targets[0], ast.Name)
            and isinstance(n.value, ast.Constant)}
    self.assertIs(opts["V6_R2_ACTIVE_CAN"], False)


if __name__ == "__main__":
  unittest.main()
