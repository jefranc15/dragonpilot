"""Native Nov 5 20Hz release regression for the R1 stale-handoff policy.

Five observed V5.0 moving BRAKING->READY transitions all had positive
CarControl.Actuators.accel, while the planner continued to request braking.
This is OFFROAD audit only; these tests do not actuate the car.
"""
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from longitudinal_r1_policy import should_abort_handoff

# (route, segment, release_time_s, true PID accel, planner brake)
RELEASES = [
    ("11:32", 3, 59.225, 1.249997, .264818),
    ("11:32", 11, 32.913, .824078, .121372),
    ("14:45", 12, 11.518, 1.694795, .121976),
    ("14:45", 26, 19.209, 1.001135, .089339),
    ("14:45", 26, 28.459, 1.037571, .187280),
]

class TestR1CorrectedReleaseInputs(unittest.TestCase):
  def test_no_observed_release_satisfies_stale_handoff_abort(self):
    for route, segment, t, pid_accel, planner_brake in RELEASES:
      with self.subTest(route=route, segment=segment, t=t):
        self.assertGreater(pid_accel, .10)
        self.assertFalse(should_abort_handoff(
          allowed=True, braking=True, curve=False, hold=False,
          plan_fresh=True, lead_relevant=True, lead_status=True,
          planner_brake=planner_brake, pid_accel=pid_accel,
        ))

  def test_abort_still_works_for_actual_negative_pid(self):
    self.assertTrue(should_abort_handoff(
      allowed=True, braking=True, curve=False, hold=False,
      plan_fresh=True, lead_relevant=True, lead_status=True,
      planner_brake=.18, pid_accel=-.10,
    ))

if __name__ == "__main__":
  unittest.main()
