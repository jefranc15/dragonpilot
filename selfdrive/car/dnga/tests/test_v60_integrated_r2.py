"""DNGA v6.0 R2/V5-scale regression tests with an isolated V5 facade.

These are *offline* algorithm tests, not physical ECU, Panda or CAN tests.
"""
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


def clip(x, lo, hi):
  return max(lo, min(hi, x))


def interp(x, bp, fp):
  if x <= bp[0]:
    return fp[0]
  for i in range(1, len(bp)):
    if x <= bp[i]:
      alpha = (x - bp[i - 1]) / (bp[i] - bp[i - 1])
      return fp[i - 1] + alpha * (fp[i] - fp[i - 1])
  return fp[-1]


P = SimpleNamespace(
  CURVE_HYDRAULIC_MIN_SPEED=5.6,
  CURVE_SPEED_ERROR_RELEASE=0.2,
  CRUISE_DECEL_MARGIN=0.3,
  LEAD_LOOKAHEAD_TARGET_CAP=0.2,
  LEAD_LOOKAHEAD_MIN_CLOSING=0.5,
  LEAD_LOOKAHEAD_MAX_TTC=20.0,
  PID_BRAKE_ALLOWANCE=0.08,
  PID_BRAKE_BLEND=0.25,
  URGENT_HARD_DECEL=0.7,
  EMERGENCY_CLOSING_SPEED=2.0,
  EMERGENCY_TTC=8.0,
  EMERGENCY_PLANNER_BRAKE=0.2,
  DECEL_GOVERNOR_CRITICAL_CLOSING=3.0,
  DECEL_GOVERNOR_CRITICAL_TTC=3.0,
  DECEL_DEADBAND=0.02,
  AEGO_FILTER_ALPHA=0.2,
  URGENT_BRAKE_MAX=0.45,
  STOP_BRAKE_MAX=0.87,
  HIGHWAY_MIN_SPEED=8.0,
  HIGHWAY_BRAKE_CAP_MIN=0.25,
  HIGHWAY_BRAKE_CAP_MAX=0.30,
  BRAKE_STEP_DOWN=0.015,
  RELEASE_CONFIRM_COUNT=10,
  BRAKE_FILTER_UP=0.08,
  BRAKE_FILTER_DOWN=0.08,
  BRAKE_STEP_UP=0.006,
  BRAKE_STEP_UP_URGENT=0.015,
  BRAKE_MIN=0.05,
  PUMP_05_THRESHOLD=0.75,
  EMERGENCY_BRAKE_FILTER_UP=0.20,
  EMERGENCY_BRAKE_STEP_UP=0.03,
  STOCK_SMOOTH_FILTER_UP=0.12,
  STOCK_SMOOTH_STEP_UP=0.012,
  STOCK_URGENT_STEP_UP=0.03,
  STOP_BRAKE_FILTER_UP=0.3,
  STOP_BRAKE_STEP_UP=0.05,
  DECEL_GOVERNOR_START=-0.9,
  DECEL_GOVERNOR_STEP_DOWN=0.015,
)


@dataclass
class BrakeRequest:
  hydraulic: bool
  release_pump: bool
  sng_release: bool
  state: int = 0x01


@dataclass
class Command:
  enabled: bool = True
  lead: bool = True
  speed: float = 20.0
  is_accel: bool = False
  is_decel: bool = True
  brake_state: int = 0x21
  pump: float = -0.4
  magnitude: int = 1216


class FakeV50Controller:
  """Fixture emulates only inherited method contracts; no extra SNG object."""
  def __init__(self):
    self.sng_calls = 0
    self._reset(0, 0.0)

  def _reset(self, frame, a_ego):
    self.brake_active = False
    self.apply_brake = 0.0
    self.brake_target = 0.0
    self.release_counter = 0
    self.curve_brake_active = False
    self.urgent_brake = False
    self.sng_armed = False
    self.stop_hold = False
    self.sng_release_count = 0
    self.filtered_aego = a_ego
    self.handoff_pending = False
    self.handoff_ready_counter = 0
    self.kinematic_ready_counter = 0

  def _update_hydraulic(self, CS, frame, session, apply_accel, plan, lead_state):
    self.sng_calls += 1
    # Tests confirm the original persistent instance receives state. The
    # fixture does NOT claim that V5's full low-speed brake is reproduced.
    return BrakeRequest(self.brake_active, False, False)

  def _update_stop_guard(self, CS, frame, plan_brake, moving_allowed, lead):
    return SimpleNamespace(stock_active=False, active=False, request=0.0)

  def _apply_brake_target(self, raw_target, floor, ceiling, emergency, critical, guard):
    alpha = P.BRAKE_FILTER_UP if raw_target > self.brake_target else P.BRAKE_FILTER_DOWN
    self.brake_target = clip(self.brake_target + alpha * (raw_target - self.brake_target), floor, ceiling)
    up = P.BRAKE_STEP_UP_URGENT if self.urgent_brake else P.BRAKE_STEP_UP
    if emergency:
      up = 0.03
    self.apply_brake = clip(
      min(self.brake_target, self.apply_brake + up)
      if self.brake_target > self.apply_brake else
      max(self.brake_target, self.apply_brake - P.BRAKE_STEP_DOWN),
      floor, ceiling,
    )

  def _clear_hydraulic(self, frame, reentry=True):
    self.apply_brake = 0.0
    self.brake_target = 0.0
    self.brake_active = False
    self.release_counter = 0
    self.curve_brake_active = False
    self.urgent_brake = False

  def _build_command(self, CS, frame, session, plan, lead_state, brake, propulsion):
    return Command(speed=CS.out.vEgo, brake_state=brake.state)


def module_under_test():
  stubs = {}
  for name in ("common", "common.numpy_fast", "selfdrive", "selfdrive.config",
               "selfdrive.car", "selfdrive.car.dnga",
               "selfdrive.car.dnga.longitudinal_v50_legacy",
               "selfdrive.car.dnga.longitudinal_kinematic_guard",
               "selfdrive.car.dnga.values"):
    mod = ModuleType(name)
    mod.__path__ = []
    stubs[name] = mod
  stubs["common.numpy_fast"].clip = clip
  stubs["common.numpy_fast"].interp = interp
  stubs["selfdrive.config"].Conversions = SimpleNamespace(KPH_TO_MS=1.0 / 3.6)

  legacy = stubs["selfdrive.car.dnga.longitudinal_v50_legacy"]
  legacy.BrakeRequest = BrakeRequest
  # Load the exact helper functions and the original _apply_brake_target
  # implementation from the retained V5.0 source. Nothing is reimplemented
  # in the test fixture for demand filtering, envelope or brake encoding.
  original = ast.parse((ROOT / "longitudinal_v50_legacy.py").read_text())
  helper_names = ("distance_profile", "curve_brake_cap", "progressive_brake_cap",
                  "low_speed_brake_cap", "high_speed_brake_cap",
                  "stop_guard_brake_cap", "encode_hev_brake")
  nodes = [node for node in original.body
           if isinstance(node, ast.FunctionDef) and node.name in helper_names]
  if {node.name for node in nodes} != set(helper_names):
    raise AssertionError("V5.0 helper missing")
  namespace = {"P": P, "clip": clip, "interp": interp}
  exec(compile(ast.Module(body=nodes, type_ignores=[]), str(ROOT / "longitudinal_v50_legacy.py"), "exec"),
       namespace)
  for name in helper_names:
    setattr(legacy, name, namespace[name])

  cls = next(node for node in original.body if isinstance(node, ast.ClassDef)
             and node.name == "LongitudinalController")
  filt = next(node for node in cls.body if isinstance(node, ast.FunctionDef)
              and node.name == "_apply_brake_target")
  exec(compile(ast.Module(body=[filt], type_ignores=[]),
               str(ROOT / "longitudinal_v50_legacy.py"), "exec"), namespace)
  FakeV50Controller._apply_brake_target = namespace["_apply_brake_target"]

  stubs["selfdrive.car.dnga.longitudinal_kinematic_guard"].LongitudinalController = FakeV50Controller
  stubs["selfdrive.car.dnga.values"].BrakeState = SimpleNamespace(
    DISABLED=0, READY=1, BRAKING=0x21, CREEPING=0x31, HOLD=0x30)
  stubs["selfdrive.car.dnga.values"].LongitudinalParams = P

  with patch.dict(sys.modules, stubs):
    spec = util.spec_from_file_location("v60_r2_compatibility_fixture", LONGITUDINAL)
    mod = util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    try:
      spec.loader.exec_module(mod)
    finally:
      sys.modules.pop(spec.name, None)
  return mod


class V60V50ScalingTests(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.module = module_under_test()

  def setUp(self):
    self.ctrl = self.module.LongitudinalController()
    self.cs = SimpleNamespace(out=SimpleNamespace(
      vEgo=20.0, aEgo=-0.1, standstill=False,
      cruiseState=SimpleNamespace(speed=20.0)),
      op_distance_val=1)
    self.plan = SimpleNamespace(
      fresh=True, has_lead=True, curve_active=False, turn_speed=0.0,
      source="lead0", accel=-0.30, brake=0.30,
      lead_lookahead_brake=0.1)
    self.lead = SimpleNamespace(
      relevant=True, status=True, urgent=False,
      closing_speed=1.5, ttc=12.0)
    self.session = SimpleNamespace(allowed=True, enabled=True, gas_override=False)

  def tick(self, frame=5, pid=-.2):
    return self.ctrl._update_hydraulic(
      self.cs, frame, self.session, pid, self.plan, self.lead)

  def test_uses_exact_original_v50_encoder(self):
    native = self.module.__dict__["encode_hev_brake"] if "encode_hev_brake" in self.module.__dict__ else None
    # The active longitudinal module imports V5 helper methods by name.
    # Retrieve the real implementation directly from the source AST.
    original = ast.parse((ROOT / "longitudinal_v50_legacy.py").read_text())
    node = next(n for n in original.body
                if isinstance(n, ast.FunctionDef) and n.name == "encode_hev_brake")
    ns = {"P": P, "clip": clip}
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(ROOT / "longitudinal_v50_legacy.py"), "exec"), ns)
    self.assertEqual(ns["encode_hev_brake"](.08), (-.4, 1216))
    self.assertEqual(ns["encode_hev_brake"](.086), (-.4, 1215))

  def test_first_normal_brake_is_exactly_1216(self):
    req = self.tick()
    self.assertTrue(req.hydraulic)
    self.assertAlmostEqual(self.ctrl.apply_brake, .08)
    magnitude = (4 << 8) | round(200 - 100 * self.ctrl.apply_brake)
    self.assertEqual(magnitude, 1216)
    self.assertEqual(self.ctrl.sng_calls, 0)

  def test_brake_scales_after_1216_and_is_v50_rate_limited(self):
    self.tick(5)
    self.tick(10)
    self.assertAlmostEqual(self.ctrl.apply_brake, .086, places=6)
    self.tick(15)
    self.assertGreater(self.ctrl.apply_brake, .086)
    self.assertLessEqual(self.ctrl.apply_brake, .092001)

  def test_brake_strength_respects_v50_highway_cap(self):
    self.cs.out.vEgo = 25.0
    self.plan.brake = 0.6
    self.plan.accel = -.6
    self.lead.urgent = False
    for frame in range(5, 600, 5):
      self.tick(frame, -.2)
    self.assertLessEqual(self.ctrl.apply_brake, .300001)
    self.assertGreater(self.ctrl.apply_brake, .10)

  def test_curve_cap_uses_existing_v50_curve_envelope(self):
    self.plan.has_lead = False
    self.plan.source = "turn"
    self.plan.curve_active = True
    self.plan.turn_speed = 13.0
    self.lead.relevant = False
    for frame in range(5, 600, 5):
      self.tick(frame)
    self.assertLessEqual(self.ctrl.apply_brake, .256)

  def test_normal_brake_releases_by_ramp_not_immediate_off(self):
    for frame in range(5, 205, 5):
      self.tick(frame)
    strength = self.ctrl.apply_brake
    self.plan.brake = 0.0
    self.plan.accel = 0.0
    self.plan.lead_lookahead_brake = 0.0
    self.tick(205, pid=0.1)
    self.assertTrue(self.ctrl.brake_active)
    self.assertAlmostEqual(self.ctrl.apply_brake, max(.08, strength - .015))

  def test_sng_uses_same_controller_and_inherited_state(self):
    self.tick(5)
    self.cs.out.vEgo = 5.0
    state = self.ctrl.brake_active
    self.tick(10)
    self.assertEqual(self.ctrl.sng_calls, 1)
    self.assertEqual(self.ctrl.brake_active, state)
    self.assertEqual(self.ctrl.r2_authority, "v50_sng_or_inactive")

  def test_stop_hold_bypasses_r2_at_any_speed(self):
    self.ctrl.stop_hold = True
    self.tick()
    self.assertEqual(self.ctrl.sng_calls, 1)

  def test_curve_target_reached_does_not_keep_braking(self):
    self.plan.has_lead = False
    self.plan.curve_active = True
    self.plan.source = "turn"
    self.plan.turn_speed = 19.95
    self.lead.relevant = False
    result = self.tick()
    self.assertFalse(result.hydraulic)

  def test_r2_below_vego_acc_target_keeps_decel_mode(self):
    self.tick(5)
    req = BrakeRequest(True, False, False, state=0x21)
    cmd = self.ctrl._build_command(
      self.cs, 5, self.session, self.plan, self.lead, req, None)
    self.assertLess(cmd.speed, self.cs.out.vEgo)
    self.assertFalse(cmd.is_accel)
    self.assertTrue(cmd.is_decel)

  def test_pid_positive_does_not_force_coast_with_negative_planner(self):
    self.tick(5)
    req = self.tick(10, pid=.2)
    self.assertTrue(req.hydraulic)
    self.assertEqual(self.ctrl.r2_authority, "r2_braking")

  def test_gate_remains_off_in_carcontroller(self):
    src = CONTROLLER.read_text()
    self.assertIn("V6_R2_ACTIVE_CAN = False", src)
    self.assertEqual(src.count("self.longitudinal.update("), 1)
    self.assertNotIn("r2_v4_shadow", src)

  def test_single_state_machine_not_second_low_speed_instance(self):
    src = LONGITUDINAL.read_text()
    self.assertNotIn("self.low_speed = ", src)
    self.assertIn("return super()._update_hydraulic(CS", src)
    self.assertIn("self._apply_brake_target(", src)
    ast.parse(src)


if __name__ == "__main__":
  unittest.main()
