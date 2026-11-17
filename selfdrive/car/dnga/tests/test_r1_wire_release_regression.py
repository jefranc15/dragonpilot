"""Regression for the Nov 5 CarControl steer-vs-accel decoding mistake.

The 32-byte hex fixtures are exact native CarControl.Actuators data-section
snapshots recorded just before five OP 0x271 BRAKING->READY transitions.
Does not require pycapnp, a running car, or external log files.
"""
import struct
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from longitudinal_r1_policy import should_abort_handoff

# (mono_s, 32-byte Actuators wire data, planner accel0, prior mislabeled steer)
FIXTURES = (
  (847.653,  "0000000000000000000080bfcb5c6cbd0befd83f010000000000000000000000", -.12102, -1.00000),
  (1695.333, "00000000000000004681dcbe25c922403125803f010000000000000000000000", -.08900, -.43067),
  (1704.583, "0000000000000000000080bf1deac33f20cf843f010000000000000000000000", -.18694, -1.00000),
  (1790.305, "0000000000000000dc4048be1d7b8a40e3ff9f3f010000000000000000000000", -.26453, -.19556),
  (2243.985, "00000000000000000000803f1c2d1941c7f6523f010000000000000000000000", -.11954, +1.00000),
)


class TestR1NativeActuatorRegression(unittest.TestCase):
  def test_steer_is_not_longitudinal_accel(self):
    for mono_s, hex32, planner_accel0, old_steer in FIXTURES:
      with self.subTest(mono_s=mono_s):
        payload = bytes.fromhex(hex32)
        self.assertEqual(len(payload), 32)
        steer = struct.unpack_from("<f", payload, 8)[0]
        accel = struct.unpack_from("<f", payload, 16)[0]
        self.assertAlmostEqual(steer, old_steer, delta=.0001)
        self.assertLess(planner_accel0, 0.)
        self.assertGreater(accel, .5)  # All five PID demands are positive.

        # Test the exact candidate's public external-state guard under
        # maximally permissive other flags. It cannot cancel a handoff here.
        self.assertFalse(should_abort_handoff(
          allowed=True, braking=True, curve=False, hold=False,
          plan_fresh=True, lead_relevant=True, lead_status=True,
          planner_brake=-planner_accel0, pid_accel=accel,
        ))


if __name__ == "__main__":
  unittest.main()
