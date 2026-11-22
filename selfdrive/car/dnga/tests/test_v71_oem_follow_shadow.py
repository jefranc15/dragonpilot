"""V7.1 OEM-follow read-only arbitration: synthetic regression scenarios.

These tests do not replay a live ECU; they check fail-closed diagnostic
classification of previously observed risk patterns. No CAN/Panda dependency.
"""
import pathlib
import sys
import unittest
from dataclasses import replace

DNGA = pathlib.Path(__file__).resolve().parent.parent
ROOT = DNGA.parents[2]
sys.path.insert(0, str(DNGA))
from oem_follow_shadow import CameraSnapshot, OEMFollowShadow, TurnSnapshot
from stock_long_shadow import StockBrakeShadow


def camera(frame=100, **changes):
  defaults = dict(
    brake_rx_frame=frame, acc_rx_frame=frame,
    brake_state=0x01, brake_magnitude=200, pump=0.0,
    enabled=True, lead=False, is_accel=True, is_decel=False,
    desired_speed_kph=90.0, set_speed_kph=90.0,
  )
  defaults.update(changes)
  return CameraSnapshot(**defaults)


def turn(frame=100, **changes):
  defaults = dict(
    rx_frame=frame, source="turn", has_lead=False,
    turn_speed_mps=15.0, near_accel=-0.2,
    lookahead_accel=-0.3,
  )
  defaults.update(changes)
  return TurnSnapshot(**defaults)


def assess(shadow, frame=100, cam=None, plan=None, **kwargs):
  return shadow.observe(
    frame, cam if cam is not None else camera(frame),
    plan if plan is not None else turn(frame),
    ego_speed_mps=25.0, ego_accel_mps2=0.0, **kwargs,
  )


class CameraPriorityTests(unittest.TestCase):
  def test_factory_lead_wins_over_op_turn(self):
    d = assess(OEMFollowShadow(), cam=camera(100, lead=True))
    self.assertEqual(d.mode, "OEM_LEAD")
    self.assertEqual(d.hypothetical_curve_target_kph, 90.0)
    self.assertTrue(d.camera_lead)

  def test_oem_braking_wins_even_when_accel_bit_is_set(self):
    c = camera(100, brake_state=0x21, brake_magnitude=173,
               pump=0.5, is_decel=True)
    d = assess(OEMFollowShadow(), cam=c)
    self.assertEqual(d.mode, "OEM_DECEL")
    self.assertAlmostEqual(d.camera_brake_strength, 0.27)

  def test_driver_gas_override(self):
    self.assertEqual(assess(OEMFollowShadow(), gas_pressed=True).mode, "DRIVER_OVERRIDE")

  def test_driver_brake_override(self):
    self.assertEqual(assess(OEMFollowShadow(), brake_pressed=True).mode, "DRIVER_OVERRIDE")

  def test_factory_disabled_no_curve(self):
    d = assess(OEMFollowShadow(), cam=camera(100, enabled=False))
    self.assertEqual(d.mode, "OEM_INACTIVE")
    self.assertEqual(d.hypothetical_curve_target_kph, d.factory_acc_kph)

  def test_camera_stale_rejected(self):
    d = assess(OEMFollowShadow(), cam=camera(100, acc_rx_frame=70))
    self.assertEqual(d.mode, "INVALID_CAMERA_PAIR")

  def test_camera_time_skew_rejected(self):
    d = assess(OEMFollowShadow(), cam=camera(100, brake_rx_frame=90, acc_rx_frame=100))
    self.assertEqual(d.mode, "INVALID_CAMERA_PAIR")

  def test_unknown_state_rejected(self):
    self.assertEqual(assess(OEMFollowShadow(), cam=camera(100, brake_state=0x99)).mode, "INVALID_CAMERA_PAIR")

  def test_duplicate_frame_rejected(self):
    s = OEMFollowShadow()
    assess(s)
    with self.assertRaises(ValueError):
      assess(s)

  def test_no_fresh_plan_uses_factory(self):
    d = assess(OEMFollowShadow(), plan=TurnSnapshot())
    self.assertEqual(d.mode, "NO_FRESH_PLAN")

  def test_op_lead_not_promoted_to_factory_lead(self):
    d = assess(OEMFollowShadow(), plan=turn(has_lead=True, source="lead0"))
    self.assertEqual(d.mode, "LEAD_SOURCE_DISAGREEMENT")
    self.assertFalse(d.camera_lead)

  def test_normal_cruise_without_turn(self):
    d = assess(OEMFollowShadow(), plan=turn(source="cruise"))
    self.assertEqual(d.mode, "OEM_CRUISE")


class CurveArbitrationTests(unittest.TestCase):
  def test_nov12_like_factory_accel_versus_op_turn(self):
    d = assess(OEMFollowShadow())
    self.assertEqual(d.mode, "CURVE_PROPULSION_CONFLICT")
    self.assertEqual(d.hypothetical_curve_target_kph, 90.0)
    self.assertFalse(d.transmittable)

  def test_oem_no_decel_turn_not_authorized(self):
    d = assess(OEMFollowShadow(), cam=camera(100, is_accel=False))
    self.assertEqual(d.mode, "CURVE_UNAUTHORIZED")

  def test_oem_decel_aligned_turn_only_shadow(self):
    s = OEMFollowShadow()
    for frame in (100, 105, 110):
      d = assess(
        s, frame, cam=camera(frame, is_accel=False, is_decel=True),
        plan=turn(frame),
      )
    self.assertEqual(d.mode, "CURVE_SHADOW_CANDIDATE")
    self.assertAlmostEqual(d.hypothetical_curve_target_kph, 54.0)
    self.assertFalse(d.transmittable)

  def test_requires_three_confirmed_samples(self):
    s = OEMFollowShadow()
    for frame in (100, 105):
      d = assess(s, frame, cam=camera(frame, is_accel=False, is_decel=True), plan=turn(frame))
      self.assertEqual(d.hypothetical_curve_target_kph, 90.0)

  def test_vision_speed_zero_invalid(self):
    d = assess(OEMFollowShadow(), plan=turn(turn_speed_mps=0))
    self.assertEqual(d.mode, "OEM_CRUISE")

  def test_no_near_future_braking_invalid(self):
    d = assess(OEMFollowShadow(), plan=turn(near_accel=0.2, lookahead_accel=0.1))
    self.assertEqual(d.mode, "OEM_CRUISE")

  def test_turn_speed_above_vehicle_invalid(self):
    d = assess(OEMFollowShadow(), plan=turn(turn_speed_mps=27))
    self.assertEqual(d.mode, "OEM_CRUISE")

  def test_plan_age_expired(self):
    d = assess(OEMFollowShadow(), plan=turn(frame=0))
    self.assertEqual(d.mode, "STALE_TURN_PLAN")


class BrakeReleaseTests(unittest.TestCase):
  def test_nov17_like_surge_warning_after_ready(self):
    s = OEMFollowShadow()
    assess(s, 100, cam=camera(100, brake_state=0x21, brake_magnitude=175,
                               pump=0.6, is_accel=False, is_decel=True))
    d = s.observe(
      105, camera(105, is_accel=False, is_decel=False),
      turn(105, source="cruise"), ego_speed_mps=17.8,
      ego_accel_mps2=1.89,
    )
    self.assertEqual(d.mode, "POST_BRAKE_SURGE_WARNING")
    self.assertFalse(d.transmittable)

  def test_release_guard_observes_not_commands(self):
    s = OEMFollowShadow()
    assess(s, 100, cam=camera(100, brake_state=0x21, brake_magnitude=180,
                               is_accel=False, is_decel=True))
    d = assess(s, 105, cam=camera(105, is_accel=False), plan=turn(105))
    self.assertEqual(d.mode, "RELEASE_MONITOR")
    self.assertEqual(d.hypothetical_curve_target_kph, d.factory_acc_kph)

  def test_factory_lead_during_release_still_prioritized(self):
    s = OEMFollowShadow()
    assess(s, 100, cam=camera(100, brake_state=0x21, brake_magnitude=180, is_decel=True))
    d = assess(s, 105, cam=camera(105, lead=True))
    self.assertEqual(d.mode, "OEM_LEAD")

  def test_hypothetical_ramp_flags_major_underbraking(self):
    s, brake_s = OEMFollowShadow(), StockBrakeShadow()
    c = camera(100, brake_state=0x21, brake_magnitude=173, pump=.6,
               is_accel=False, is_decel=True, lead=True)
    b = brake_s.observe(100, 100, 0x21, 173)
    d = assess(s, cam=c, brake_shadow=b)
    self.assertGreater(d.weakening_fraction, 0.8)
    self.assertIn("shadow_proposal_weakens_OEM_braking", d.reason)
    self.assertLess(d.hypothetical_pump, d.camera_pump)
    self.assertFalse(d.transmittable)


class SourceWiringTests(unittest.TestCase):
  def test_controller_sends_steering_only(self):
    src = (DNGA / "carcontroller.py").read_text()
    for forbidden in ("dnga_create_brake_command", "dnga_create_accel_command", "LongitudinalController", "dnga_create_hud"):
      self.assertNotIn(forbidden, src)
    self.assertIn("create_can_steer_command(", src)
    self.assertIn("OEMFollowShadow()", src)
    self.assertIn("self.oem_follow_shadow.observe(", src)

  def test_factory_forwarding_unchanged(self):
    src = (ROOT / "panda/board/safety/safety_dnga_stock.h").read_text()
    self.assertIn("GET_ADDR(to_send) == 0x1D0", src)
    self.assertIn("GET_ADDR(to_fwd) == 0x1D0 ? -1 : 0", src)
    self.assertIn("return GET_ADDR(to_send) == 0x1D0", src)

  def test_no_can_encoder_in_shadow(self):
    src = (DNGA / "oem_follow_shadow.py").read_text()
    self.assertNotIn("make_can_msg(", src)
    self.assertNotIn("sendcan", src)
    self.assertNotIn("CANPacker", src)


if __name__ == "__main__":
  unittest.main()
