"""Pure-command tests for experimental R2 V4 actuation.

No hardware or CAN transmission. Exact LongitudinalCommand-shaped dataclass
objects are used so dataclasses.replace follows the production path.
"""
from dataclasses import dataclass
from types import SimpleNamespace
import unittest

from selfdrive.car.dnga.r2_v4_experimental_actuator import (
  R2V4ExperimentalActuator, BRAKING, READY, CREEPING, HOLD, DISABLED,
  magnitude_strength,
)


@dataclass
class Command:
  enabled: bool = True
  lead: bool = False
  speed: float = 20.0
  is_accel: bool = True
  is_decel: bool = False
  brake_state: int = READY
  pump: float = 0.0
  magnitude: int = 200


def proposal(state=BRAKING, magnitude=1216, target_kph=71.82,
             remaining=0, gap=0.18, is_accel=False):
  return SimpleNamespace(brake_state=state, magnitude=magnitude,
                         target_kph=target_kph, release_frames_left=remaining,
                         target_gap_kph=gap, is_accel=is_accel)


class TestR2V4Actuator(unittest.TestCase):

  def setUp(self):
    self.ctrl = R2V4ExperimentalActuator()

  def step(self, cmd=None, prop=None, **kwargs):
    args = dict(enabled=True, speed_mps=20.0, override=False,
                urgent=False, engagement_guard=False)
    args.update(kwargs)
    return self.ctrl.apply(cmd or Command(), prop or proposal(), **args)

  def test_brake_engages_and_commands_below_ego(self):
    out, stat = self.step()
    self.assertEqual(out.brake_state, BRAKING)
    self.assertEqual(out.magnitude, 1216)
    self.assertEqual(out.pump, -0.4)
    self.assertLess(out.speed, 20.0)
    self.assertTrue(out.is_decel)
    self.assertFalse(out.is_accel)
    self.assertTrue(stat.active)
    self.assertLess(stat.expected_275_raw, 0)

  def test_base_strong_brake_not_weakened(self):
    orig = Command(brake_state=BRAKING, magnitude=(4 << 8) | 150, pump=-0.4,
                   speed=20.0, is_accel=False, is_decel=True)
    out, _ = self.step(orig, proposal(magnitude=1216))
    self.assertGreaterEqual(magnitude_strength(out.magnitude),
                            magnitude_strength(orig.magnitude))

  def test_raised_demand_is_slewed(self):
    self.step()
    out, _ = self.step(prop=proposal(magnitude=(4 << 8) | 140))
    self.assertLessEqual(magnitude_strength(out.magnitude), 0.101)

  def test_emergency_base_brake_not_weakened(self):
    orig = Command(brake_state=BRAKING, magnitude=(5 << 8) | 114,
                   pump=-0.5, is_accel=False, is_decel=True)
    out, _ = self.step(orig, proposal(magnitude=1216))
    self.assertGreaterEqual(magnitude_strength(out.magnitude), 0.85)

  def test_release_ready_is_not_accel(self):
    self.step()
    out, stat = self.step(prop=proposal(
      state=READY, magnitude=1224, remaining=23,
      target_kph=71.0, gap=1.0, is_accel=False))
    self.assertEqual((out.brake_state, out.magnitude, out.pump),
                     (READY, 1224, -0.4))
    self.assertFalse(out.is_accel)
    self.assertTrue(out.is_decel)
    self.assertEqual(stat.reason, "v4_release")

  def test_ready_does_not_cancel_existing_v5_brake(self):
    orig = Command(brake_state=BRAKING, magnitude=1180,
                   pump=-0.4, is_accel=False, is_decel=True)
    out, stat = self.step(orig, proposal(state=READY, gap=0.0))
    self.assertEqual(out, orig)
    self.assertFalse(stat.active)

  def test_accel_veto_when_v4_does_not_allow(self):
    out, stat = self.step(prop=proposal(state=READY, gap=0.0, is_accel=False))
    self.assertFalse(out.is_accel)
    self.assertFalse(out.is_decel)
    self.assertEqual(stat.reason, "v4_accel_veto")

  def test_driver_override_returns_original(self):
    orig = Command()
    out, stat = self.step(orig, override=True)
    self.assertEqual(out, orig)
    self.assertFalse(stat.active)

  def test_disengagement_returns_original(self):
    orig = Command(enabled=False, brake_state=DISABLED)
    out, stat = self.step(orig, enabled=False)
    self.assertEqual(out, orig)
    self.assertFalse(stat.active)

  def test_steady_hold_is_preserved(self):
    orig = Command(brake_state=HOLD, magnitude=1224, pump=-0.4)
    out, stat = self.step(orig)
    self.assertEqual(out, orig)
    self.assertFalse(stat.active)

  def test_creep_brake_is_preserved(self):
    orig = Command(brake_state=CREEPING, magnitude=1190, pump=-0.4)
    out, stat = self.step(orig)
    self.assertEqual(out, orig)
    self.assertFalse(stat.active)

  def test_low_speed_keeps_existing_controller(self):
    out, stat = self.step(speed_mps=3.0)
    self.assertEqual(out.brake_state, READY)
    self.assertFalse(stat.active)

  def test_engagement_guard(self):
    out, stat = self.step(engagement_guard=True)
    self.assertEqual(out.brake_state, READY)
    self.assertFalse(stat.active)

  def test_absent_proposal_is_fallback(self):
    cmd = Command()
    out, stat = self.ctrl.apply(cmd, None, enabled=True, speed_mps=20.0)
    self.assertEqual(out, cmd)
    self.assertFalse(stat.active)

  def test_target_return_remains_decel(self):
    out, stat = self.step(prop=proposal(state=READY, gap=0.10,
                                        target_kph=71.9))
    self.assertTrue(out.is_decel)
    self.assertFalse(out.is_accel)

  def test_0x273_target_never_exceeds_vehicle_speed_while_braking(self):
    out, _ = self.step(prop=proposal(target_kph=75.0))
    self.assertLessEqual(out.speed, 20.0)

if __name__ == "__main__":
  unittest.main()
