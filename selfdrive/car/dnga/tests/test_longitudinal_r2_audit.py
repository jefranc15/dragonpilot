"""Self-contained Nov 5 R2 release-audit tests, not vehicle-controller tests."""
import unittest

RELEASES = (
  # segment, PID accel, planner accel, reentry gap in seconds
  ('11:32--3', 1.249997, -.264530, .606152),
  ('11:32--11', .824078, -.119535, .197537),
  ('14:45--12', 1.694795, -.121022, .598962),
  ('14:45--26a', 1.001135, -.088998, .599818),
  ('14:45--26b', 1.037571, -.186942, .600731),
)


def r2_release_audit(pid_accel, planner_accel, gap, *, braking=True, lead=True):
  """Observation-only tag. Never used to alter actuator commands."""
  mismatch = braking and lead and pid_accel > .1 and planner_accel < 0.
  short_bypass = gap < .4
  return dict(planner_pid_disagreement=bool(mismatch),
              bypass_candidate=bool(short_bypass))


class TestR2NativeReleaseAudit(unittest.TestCase):
  def test_five_release_cases_are_planner_pid_disagreement(self):
    for seg, pid, planner, gap in RELEASES:
      with self.subTest(seg=seg):
        q = r2_release_audit(pid, planner, gap)
        self.assertTrue(q['planner_pid_disagreement'])
        self.assertEqual(q['bypass_candidate'], gap < .4)

  def test_does_not_reclassify_matching_brake_demand(self):
    self.assertFalse(r2_release_audit(-.3, -.4, .6)['planner_pid_disagreement'])
    self.assertFalse(r2_release_audit(.3, +.1, .6)['planner_pid_disagreement'])
    self.assertFalse(r2_release_audit(.3, -.1, .6, lead=False)['planner_pid_disagreement'])

  def test_probe_has_no_control_override(self):
    from pathlib import Path
    src = (Path(__file__).resolve().parents[1]/'longitudinal_r2_instrumented.py').read_text()
    self.assertNotIn('def _build_command(', src)
    self.assertNotIn('def _update_propulsion(', src)
    self.assertNotIn('command.is_accel =', src)
    self.assertIn('super()._update_hydraulic(', src)
    self.assertIn('return super()._clear_hydraulic(', src)


if __name__ == '__main__':
  unittest.main()
