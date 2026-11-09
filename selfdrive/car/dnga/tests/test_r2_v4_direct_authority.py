"""Bench-only regression tests for direct R2 command authority; never transmits CAN."""
from dataclasses import dataclass
from types import SimpleNamespace
import unittest

from selfdrive.car.dnga.r2_v4_direct_authority import (
  R2V4DirectAuthority, BRAKING, READY, HOLD, CREEPING, encode, strength_of,
)


@dataclass
class Cmd:
  enabled: bool = True
  lead: bool = True
  speed: float = 20.0
  is_accel: bool = False
  is_decel: bool = False
  brake_state: int = READY
  pump: float = 0.0
  magnitude: int = 200


def prop(state=BRAKING, magnitude=1216, gap=0.18, target=71.82,
         is_accel=False, pump=None, missing=False):
  if pump is None:
    pump = -0.4 if magnitude == 1224 else 0.0
  return SimpleNamespace(brake_state=state, magnitude=magnitude,
                         pump=pump, target_gap_kph=gap, target_kph=target,
                         is_accel=is_accel, missing_275_response=missing)


class DirectAuthorityTests(unittest.TestCase):
  def setUp(self):
    self.r2 = R2V4DirectAuthority()

  def run_step(self, base=None, candidate=None, **params):
    args = dict(enabled=True, speed_mps=20.0, override=False,
                urgent=False, engagement_guard=False, feedback_fresh=True,
                friction_active=False)
    args.update(params)
    return self.r2.apply(base if base is not None else Cmd(),
                         candidate if candidate is not None else prop(), **args)

  def test_first_brake_is_1216_even_if_v50_is_coasting(self):
    out, meta = self.run_step(Cmd(is_accel=True, speed=20.6))
    self.assertEqual((out.brake_state, out.magnitude, out.pump),
                     (BRAKING, 1216, -0.4))
    self.assertEqual(meta.authority, "r2_braking")
    self.assertFalse(out.is_accel)
    self.assertTrue(out.is_decel)
    self.assertLess(out.speed, 20.0)

  def test_normal_r2_does_not_copy_v50_coast_or_stronger_brake(self):
    base = Cmd(brake_state=BRAKING, magnitude=1180, pump=-0.4,
               is_accel=False, is_decel=True)
    out, meta = self.run_step(base, prop(magnitude=1190))
    self.assertEqual(out.magnitude, 1216)
    self.assertEqual(meta.authority, "r2_braking")

  def test_normal_step_up_is_limited(self):
    self.run_step()
    out, _ = self.run_step(candidate=prop(magnitude=1140))
    self.assertEqual(round(strength_of(out.magnitude), 2), 0.10)

  def test_staged_release_uses_stock_ready_framing(self):
    self.run_step()
    out, meta = self.run_step(
      Cmd(brake_state=BRAKING, magnitude=1190, pump=-0.4),
      prop(state=READY, magnitude=1224, pump=-0.4, gap=1.0, target=71.0))
    self.assertEqual((out.brake_state, out.magnitude, out.pump),
                     (READY, 1224, -0.4))
    self.assertEqual(meta.authority, "r2_release")
    self.assertFalse(out.is_accel)

  def test_r2_can_release_even_if_v50_wants_braking(self):
    self.run_step()
    out, meta = self.run_step(
      Cmd(brake_state=BRAKING, magnitude=1190, pump=-0.4),
      prop(state=READY, magnitude=1224, gap=1.0, target=71.0))
    self.assertEqual(out.brake_state, READY)
    self.assertEqual(meta.reason, "stock_style_ready")

  def test_r2_ready_does_not_brake_if_no_real_demand(self):
    out, meta = self.run_step(candidate=prop(state=READY, magnitude=200, gap=0.0))
    self.assertEqual((out.brake_state, out.magnitude, out.pump),
                     (READY, 200, 0.0))
    self.assertEqual(meta.authority, "r2_ready")

  def test_accel_needs_base_and_r2_agreement_and_fresh_feedback(self):
    base = Cmd(is_accel=True, speed=20.2)
    ready = prop(state=READY, magnitude=200, gap=0.0,
                 is_accel=True)
    out, _ = self.run_step(base, ready)
    self.assertTrue(out.is_accel)
    self.assertEqual(out.speed, 20.2)
    out, meta = self.run_step(base, ready, feedback_fresh=False)
    self.assertFalse(out.is_accel)
    self.assertEqual(meta.feedback_fault, "stale_brake_feedback")
    out, _ = self.run_step(base, ready, friction_active=True)
    self.assertFalse(out.is_accel)

  def test_missing_275_is_visible_not_treated_as_regen_measurement(self):
    out, meta = self.run_step(candidate=prop(missing=True))
    self.assertEqual(out.brake_state, BRAKING)
    self.assertEqual(meta.feedback_fault, "no_expected_275_response")

  def test_emergency_floor_is_not_weakened(self):
    base = Cmd(brake_state=BRAKING, magnitude=1140, pump=-0.4,
               is_accel=False, is_decel=True)
    out, _ = self.run_step(base, prop(), urgent=True)
    self.assertGreaterEqual(strength_of(out.magnitude), strength_of(base.magnitude))

  def test_urgent_v50_brake_prevents_r2_ready_release(self):
    self.run_step()
    base = Cmd(brake_state=BRAKING, magnitude=1150)
    out, meta = self.run_step(base, prop(state=READY, magnitude=1224), urgent=True)
    self.assertEqual(out, base)
    self.assertEqual(meta.reason, "urgent_brake_fallback")

  def test_stop_and_go_unchanged(self):
    for state in (HOLD, CREEPING):
      self.r2.reset()
      base = Cmd(brake_state=state, magnitude=1224)
      out, meta = self.run_step(base)
      self.assertEqual(out, base)
      self.assertEqual(meta.reason, "sng")

  def test_new_low_speed_entry_keeps_existing_controller(self):
    base = Cmd(brake_state=READY)
    out, meta = self.run_step(base, speed_mps=4.0)
    self.assertEqual(out, base)
    self.assertEqual(meta.reason, "below_entry_speed")

  def test_ongoing_brake_handoff_to_low_speed_requires_base_braking(self):
    self.run_step()
    base = Cmd(brake_state=BRAKING, magnitude=1200)
    out, meta = self.run_step(base, speed_mps=5.0)
    self.assertEqual(out, base)
    self.assertEqual(meta.reason, "moving_to_low_speed_braking")

  def test_disengage_and_pedal_override(self):
    self.run_step()
    base = Cmd(enabled=False)
    out, _ = self.run_step(base, enabled=False)
    self.assertEqual(out, base)
    self.assertFalse(self.r2.owned_braking)
    base = Cmd()
    out, _ = self.run_step(base, override=True)
    self.assertEqual(out, base)

  def test_engagement_guard_prevents_early_entry(self):
    base = Cmd()
    out, meta = self.run_step(base, engagement_guard=True)
    self.assertEqual(out, base)
    self.assertEqual(meta.reason, "engagement_guard")

  def test_invalid_or_missing_plan_falls_back(self):
    base = Cmd()
    out, meta = self.r2.apply(base, None, enabled=True, speed_mps=20.0)
    self.assertEqual(out, base)
    self.assertEqual(meta.reason, "invalid_or_stale_plan")

  def test_encoding(self):
    self.assertEqual(encode(0.08), (1216, -0.4))
    self.assertEqual(encode(0.75)[1], -0.5)


if __name__ == "__main__":
  unittest.main()
