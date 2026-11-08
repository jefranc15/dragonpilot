"""Regression checks for the non-actuating R2V4 shadow candidate."""
import unittest

from selfdrive.car.dnga.r2_v4_shadow_controller import (
  R2V4ShadowController, BRAKING, READY, DISABLED,
)


def sample(c, frame, **overrides):
  inputs = dict(frame=frame, v_ego_mps=20.0,
                planned_accel=-0.08, preview_accel=-0.08,
                downstream_pid=-0.10, measured_accel=-0.10,
                source="turn", has_lead=False, enabled=True,
                feedback_275=0, friction_08c=0, feedback_fresh=True,
                feedback_sequence=(frame, frame))
  inputs.update(overrides)
  return c.step(**inputs)


class TestR2V4Shadow(unittest.TestCase):
  def test_disabled_stays_disabled(self):
    result = sample(R2V4ShadowController(), 5, enabled=False)
    self.assertEqual(result.brake_state, DISABLED)
    self.assertFalse(result.is_accel)

  def test_weakest_brake_is_1216(self):
    result = sample(R2V4ShadowController(), 5)
    self.assertEqual(result.brake_state, BRAKING)
    self.assertEqual(result.magnitude, 1216)
    self.assertEqual(result.pump, -0.4)

  def test_stronger_brake_not_capped_at_1216(self):
    result = sample(R2V4ShadowController(), 5, planned_accel=-0.50, preview_accel=-0.50)
    self.assertEqual(result.magnitude, (4 << 8) | 150)

  def test_pump_high_demand(self):
    result = sample(R2V4ShadowController(), 5, planned_accel=-0.85, preview_accel=-0.85)
    self.assertEqual(result.pump, -0.5)
    self.assertEqual(result.magnitude, (5 << 8) | 115)

  def test_target_gap_slew(self):
    c = R2V4ShadowController()
    a = sample(c, 5, planned_accel=-0.8)
    b = sample(c, 10, planned_accel=-0.8)
    self.assertLessEqual(a.target_gap_kph, 0.18001)
    self.assertLessEqual(b.target_gap_kph - a.target_gap_kph, 0.18001)

  def test_source_bridge_requires_pid_agreement(self):
    c = R2V4ShadowController()
    sample(c, 5, planned_accel=-0.20, downstream_pid=-0.20,
           measured_accel=-0.15)
    a = sample(c, 10, source="cruise", has_lead=False,
               planned_accel=-0.20, downstream_pid=-0.20, measured_accel=-0.15)
    self.assertTrue(a.source_bridge)
    self.assertEqual(a.brake_state, BRAKING)
    b = sample(c, 15, source="cruise", has_lead=False,
               planned_accel=0.10, preview_accel=0.10,
               downstream_pid=0.10, measured_accel=-0.15)
    self.assertFalse(b.source_bridge)
    self.assertEqual(b.brake_state, READY)

  def test_source_switch_does_not_start_untrusted_braking(self):
    c = R2V4ShadowController()
    a = sample(c, 5, source="cruise", has_lead=False,
               planned_accel=-0.3, downstream_pid=-0.3)
    self.assertEqual(a.brake_state, READY)

  def test_release_uses_special_ready_then_normal(self):
    c = R2V4ShadowController()
    sample(c, 5)
    a = sample(c, 10, planned_accel=0.1, preview_accel=0.1,
               downstream_pid=0.1)
    self.assertEqual(a.brake_state, READY)
    self.assertEqual((a.magnitude, a.pump), (1224, -0.4))
    for j in range(1, 25):
      a = sample(c, 10 + j*5, planned_accel=0.1, preview_accel=0.1,
                 downstream_pid=0.1)
    self.assertEqual((a.magnitude, a.pump), (200, 0.0))

  def test_brake_feedback_never_allows_accel(self):
    c = R2V4ShadowController()
    for j in range(1, 8):
      r = sample(c, j*5, planned_accel=0.2, preview_accel=0.2,
                 downstream_pid=0.2, feedback_275=-500, friction_08c=0,
                 positive_accel_permitted=True)
      self.assertFalse(r.is_accel)
    r = sample(c, 40, planned_accel=0.2, preview_accel=0.2,
               downstream_pid=0.2, feedback_275=0, friction_08c=1,
               positive_accel_permitted=True)
    self.assertFalse(r.is_accel)

  def test_stale_feedback_blocks_accel(self):
    c = R2V4ShadowController()
    for j in range(1, 8):
      r = sample(c, j*5, planned_accel=0.2, preview_accel=0.2,
                 downstream_pid=0.2, feedback_fresh=False,
                 positive_accel_permitted=True)
      self.assertFalse(r.is_accel)

  def test_domain_warning_highway(self):
    c = R2V4ShadowController()
    r = sample(c, 5, v_ego_mps=30.0)
    self.assertTrue(r.outside_stock_calibration)

if __name__ == "__main__":
  unittest.main()
