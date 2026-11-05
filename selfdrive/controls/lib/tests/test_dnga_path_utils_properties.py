"""Extra offline-only regression tests for V5.1 E2E/lane path separation."""
import unittest

import numpy as np

from selfdrive.controls.lib.dnga_path_utils import build_dnga_paths


class FakeLanePlanner:
  def __init__(self, camera_offset, path_offset, lane_center=0., probability=1.):
    self.camera_offset = camera_offset
    self.path_offset = path_offset
    self.lane_center = lane_center
    self.probability = probability

  def get_d_path(self, v_ego, t_idxs, xyz):
    # The real V5.0 LanePlanner.get_d_path() mutates its input array.
    xyz[:, 1] = (self.probability * (self.lane_center + self.camera_offset)
                 + (1. - self.probability) * xyz[:, 1])
    xyz[:, 1] += self.path_offset
    return xyz


class TestDngaPathProperties(unittest.TestCase):
  def test_alias_regression_on_confident_lanes(self):
    raw = np.array([[0., .30, 0.], [10., .35, 0.]])
    original = raw.copy()
    lp = FakeLanePlanner(-.04, -.05)
    e2e, lane = build_dnga_paths(raw, lp, 15., np.array([0., 1.]))
    np.testing.assert_array_equal(raw, original)
    np.testing.assert_allclose(e2e[:, 1], [.21, .26])
    np.testing.assert_allclose(lane[:, 1], [-.09, -.09])
    self.assertFalse(np.shares_memory(e2e, lane))

  def test_camera_and_path_offsets_translate_both_once(self):
    raw = np.array([[0., -.1, 0.], [10., .2, 0.]])
    for probability in (0., .3, 1.):
      baseline = build_dnga_paths(raw, FakeLanePlanner(0., 0., probability=probability), 10., np.arange(2))
      changed = build_dnga_paths(raw, FakeLanePlanner(-.04, -.05, probability=probability), 10., np.arange(2))
      for baseline_path, changed_path in zip(baseline, changed):
        np.testing.assert_allclose(changed_path[:, 1] - baseline_path[:, 1], -.09)

  def test_5000_random_inputs_immutable_and_single_offset(self):
    rng = np.random.default_rng(20261105)
    for _ in range(5000):
      raw = np.column_stack((np.linspace(0, 80, 33), rng.normal(0, .4, 33), np.zeros(33)))
      before = raw.copy()
      c, p, q = rng.uniform(-.15, .15), rng.uniform(-.12, .12), rng.uniform(0, 1)
      lp = FakeLanePlanner(c, p, lane_center=rng.uniform(-1, 1), probability=q)
      e2e, lane = build_dnga_paths(raw, lp, 18., np.arange(33))
      np.testing.assert_allclose(e2e[:, 1], before[:, 1] + c + p, rtol=0, atol=1e-12)
      np.testing.assert_allclose(lane[:, 1], q * (lp.lane_center + c) + (1-q) * (before[:, 1] + c) + p, rtol=0, atol=1e-12)
      np.testing.assert_array_equal(raw, before)
      self.assertFalse(np.shares_memory(e2e, lane))
      self.assertFalse(np.shares_memory(raw, e2e))
      self.assertFalse(np.shares_memory(raw, lane))


if __name__ == '__main__':
  unittest.main()
