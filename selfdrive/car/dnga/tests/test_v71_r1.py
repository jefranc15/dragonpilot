"""R1 camera-disengagement and V6 release-anomaly regression tests.

All samples below are synthetic except the documented Nov 17 event
characteristics (t=1664.5916, aEgo up to +1.89 m/s^2, camera READY).
The original raw rlogs are NOT embedded in this repository. These tests
check decoder contracts and replay path; they do not prove real ECU safety.
"""
import pathlib
import sys
import unittest

DNGA = pathlib.Path(__file__).resolve().parent.parent
REPO = DNGA.parents[2]
sys.path.insert(0, str(DNGA))

from stock_acc_decode import dnga_rx_checksum_valid, decode_stock_acc_cmd_273
from oem_follow_shadow import CameraSnapshot, OEMFollowShadow, TurnSnapshot


def frame273(*, enabled, constant_b1=True, constant_b6=True,
             is_accel=False, is_decel=False, lead=False,
             speed_raw=0, set_speed=80):
  dat = bytearray(8)
  dat[0] = set_speed
  dat[1] = ((0x20 if enabled else 0) | (0x02 if constant_b1 else 0)
            | (0x08 if lead else 0))
  dat[2] = (speed_raw >> 8) & 0xff
  dat[3] = speed_raw & 0xff
  dat[4] = (0x40 if is_accel else 0) | (0x20 if is_decel else 0)
  dat[6] = 0x40 if constant_b6 else 0
  dat[7] = (0x273 + 8 + sum(dat[:-1])) & 0xff
  return bytes(dat)


def camera(frame, state=0x01, enabled=True, lead=False,
           is_accel=True, is_decel=False):
  return CameraSnapshot(
    brake_rx_frame=frame, acc_rx_frame=frame,
    brake_state=state, brake_magnitude=200, pump=0.0,
    enabled=enabled, lead=lead, is_accel=is_accel, is_decel=is_decel,
    desired_speed_kph=75.0 if enabled else 0.0,
    set_speed_kph=75.0,
  )


def neutral_plan():
  return TurnSnapshot(source="cruise")


class InactiveCameraDecodingTests(unittest.TestCase):
  def test_checksum_valid_inactive_all_engagement_constant_combinations(self):
    # V7.1 incorrectly rejected disabled states when either constant was 0.
    for constant_b1 in (False, True):
      for constant_b6 in (False, True):
        dat = frame273(enabled=False, constant_b1=constant_b1, constant_b6=constant_b6)
        with self.subTest(b1=constant_b1, b6=constant_b6):
          self.assertTrue(dnga_rx_checksum_valid(0x273, dat))
          self.assertEqual(decode_stock_acc_cmd_273(dat), (False, False, False, False, 0.0))

  def test_inactive_ignores_stale_mode_and_target_bits(self):
    dat = frame273(enabled=False, constant_b1=False, constant_b6=False,
                   lead=True, is_accel=True, is_decel=True, speed_raw=7000)
    self.assertEqual(decode_stock_acc_cmd_273(dat), (False, False, False, False, 0.0))

  def test_enabled_valid_camera_frame_preserves_lead_and_modes(self):
    dat = frame273(enabled=True, lead=True, is_accel=False,
                   is_decel=True, speed_raw=7150)
    self.assertEqual(decode_stock_acc_cmd_273(dat), (True, True, False, True, 71.5))

  def test_enabled_missing_byte1_constant_rejected(self):
    dat = frame273(enabled=True, constant_b1=False, constant_b6=True)
    self.assertIsNone(decode_stock_acc_cmd_273(dat))

  def test_enabled_missing_byte6_constant_rejected(self):
    dat = frame273(enabled=True, constant_b1=True, constant_b6=False)
    self.assertIsNone(decode_stock_acc_cmd_273(dat))

  def test_invalid_checksum_rejected_even_when_disabled(self):
    dat = bytearray(frame273(enabled=False))
    dat[7] ^= 0x01
    self.assertIsNone(decode_stock_acc_cmd_273(dat))

  def test_wrong_payload_length_rejected(self):
    self.assertIsNone(decode_stock_acc_cmd_273(bytes(7)))
    self.assertIsNone(decode_stock_acc_cmd_273(bytes(9)))

  def test_valid_ready_to_inactive_transition_is_immediate(self):
    # Decoder must not wait for 15-frame freshness expiry to see inactive.
    active = frame273(enabled=True, speed_raw=7000)
    inactive = frame273(enabled=False, constant_b1=False, constant_b6=False)
    self.assertTrue(decode_stock_acc_cmd_273(active)[0])
    self.assertFalse(decode_stock_acc_cmd_273(inactive)[0])

  def test_interface_caches_checksummed_inactive_and_clears_invalid(self):
    src = (DNGA / "interface.py").read_text()
    self.assertIn("from selfdrive.car.dnga.stock_acc_decode import", src)
    self.assertIn("self.CS.stock_acc_request_enabled = enabled", src)
    self.assertIn("self.CS.stock_acc_request_enabled = False", src)
    self.assertIn("self.CS.stock_acc_request_rx_frame = -1000000", src)


class HistoricV6ReleaseTests(unittest.TestCase):
  def test_documented_nov17_op_release_not_camera_release(self):
    # Summary-derived event fixture, not a raw rlog replay:
    # segment 27 t=1664.5916, factory always READY and IS_ACCEL=1,
    # OP transmitted BRAKING -> READY, aEgo peak +1.89 without pedal.
    shadow = OEMFollowShadow()
    before = shadow.observe(
      100, camera(100), neutral_plan(),
      ego_speed_mps=59.2 / 3.6, ego_accel_mps2=-0.3,
      historical_op_brake_state=0x21,
    )
    self.assertEqual(before.mode, "NO_FRESH_PLAN")
    after = shadow.observe(
      105, camera(105), neutral_plan(),
      ego_speed_mps=59.2 / 3.6, ego_accel_mps2=1.89,
      historical_op_brake_state=0x01,
    )
    self.assertEqual(after.mode, "POST_BRAKE_SURGE_WARNING")
    self.assertFalse(after.transmittable)

  def test_without_historic_op_release_there_is_no_false_release(self):
    shadow = OEMFollowShadow()
    shadow.observe(100, camera(100), neutral_plan(),
                   ego_speed_mps=16.4, ego_accel_mps2=-.3)
    result = shadow.observe(105, camera(105), neutral_plan(),
                            ego_speed_mps=16.4, ego_accel_mps2=1.89)
    self.assertEqual(result.mode, "NO_FRESH_PLAN")

  def test_camera_braking_release_still_reported(self):
    shadow = OEMFollowShadow()
    shadow.observe(100, camera(100, state=0x21, is_accel=False, is_decel=True),
                   neutral_plan(), ego_speed_mps=16.4, ego_accel_mps2=-.3)
    result = shadow.observe(105, camera(105), neutral_plan(),
                            ego_speed_mps=16.4, ego_accel_mps2=1.2)
    self.assertEqual(result.mode, "POST_BRAKE_SURGE_WARNING")

  def test_historic_op_unchanged_ready_never_arms_release(self):
    shadow = OEMFollowShadow()
    for t in (100, 105, 110):
      result = shadow.observe(
        t, camera(t), neutral_plan(),
        ego_speed_mps=16.4, ego_accel_mps2=1.89,
        historical_op_brake_state=0x01,
      )
      self.assertNotEqual(result.mode, "POST_BRAKE_SURGE_WARNING")

  def test_live_controller_does_not_inject_historical_op_brake(self):
    src = (DNGA / "carcontroller.py").read_text()
    self.assertNotIn("historical_op_brake_state=", src)
    for forbidden in ("dnga_create_brake_command", "dnga_create_accel_command",
                      "LongitudinalController", "dnga_create_hud"):
      self.assertNotIn(forbidden, src)

  def test_stocklong_panda_unchanged_source_contract(self):
    panda = (REPO / "panda/board/safety/safety_dnga_stock.h").read_text()
    self.assertIn("GET_ADDR(to_send) == 0x1D0", panda)
    self.assertIn("GET_ADDR(to_fwd) == 0x1D0 ? -1 : 0", panda)
    iface = (DNGA / "interface.py").read_text()
    self.assertIn("ret.openpilotLongitudinalControl = False", iface)


if __name__ == "__main__":
  unittest.main()
