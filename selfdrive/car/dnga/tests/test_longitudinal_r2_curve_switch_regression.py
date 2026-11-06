"""Historical 2026-11-06 curve-source flicker regression fixtures.

These cases are purely OBSERVATIONAL. They must not be used as actuator
timing targets until the exact-controller replay validates release causality.
"""
import unittest

# segment, turn->cruise relative s, 0x271 READY relative s,
# planner accel at source change, downstream accel at source change
CURVE_RELEASES = (
  ('--86-a', 37.527187, 37.994349, -0.154797, +0.027302),
  ('--86-b', 46.032966, 46.501365, -0.183898, -0.572535),
  ('--86-c', 58.883511, 59.434731, -0.318488, -0.522782),
  ('--87-a', 34.404924, 34.854211, -0.255928, -0.437333),
  ('--87-b', 43.763294, 44.273857, -0.310673, -0.468860),
)


class TestR2ObservedCurveReleaseWindows(unittest.TestCase):
  def test_all_five_no_lead_source_changes_precede_release_near_half_second(self):
    for seg, change, release, planner, pid in CURVE_RELEASES:
      with self.subTest(segment=seg):
        self.assertGreater(release, change)
        self.assertAlmostEqual(release - change, 0.5, delta=0.07)
        self.assertLess(planner, 0.0)

  def test_four_cases_still_have_negative_downstream_accel(self):
    self.assertEqual(sum(pid < 0 for _, _, _, _, pid in CURVE_RELEASES), 4)

  def test_strongest_surging_case_still_negative_at_source_switch(self):
    seg, change, release, planner, pid = CURVE_RELEASES[-1]
    self.assertEqual(seg, '--87-b')
    self.assertLess(planner, -0.30)
    self.assertLess(pid, -0.46)
    self.assertAlmostEqual(release - change, 0.510563, delta=0.002)


if __name__ == '__main__':
  unittest.main()
