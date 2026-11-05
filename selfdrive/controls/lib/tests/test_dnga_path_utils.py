"""Offline unit tests for V5.1 path frame correction and alias separation."""
import unittest

import numpy as np

from selfdrive.controls.lib.dnga_path_utils import build_dnga_paths


class TestDngaPaths(unittest.TestCase):
  class FakeLP:
    def __init__(self, camera_offset, path_offset, lane_center=0.0, probability=1.0):
      self.camera_offset = camera_offset
      self.path_offset = path_offset
      self.lane_center = lane_center
      self.probability = probability
      self.calls = 0

    def get_d_path(self, v_ego, t_idxs, xyz):
      self.calls += 1
      xyz[:, 1] = (self.probability * (self.lane_center + self.camera_offset)
                   + (1.0 - self.probability) * xyz[:, 1])
      xyz[:, 1] += self.path_offset
      return xyz

  def test_e2e_distinct_from_lane_with_confident_lane_lines(self):
    raw = np.array([[0, 0.30, 0], [10, 0.35, 0]], dtype=float)
    before = raw.copy()
    lp = self.FakeLP(camera_offset=-0.04, path_offset=-0.05)
    e2e, lane = build_dnga_paths(raw, lp, 15., np.array([0., 1.]))
    np.testing.assert_allclose(raw, before)
    np.testing.assert_allclose(e2e[:, 1], [0.21, 0.26])
    np.testing.assert_allclose(lane[:, 1], [-0.09, -0.09])
    self.assertFalse(np.shares_memory(e2e, lane))
    self.assertEqual(lp.calls, 1)

  def test_low_confidence_lane_falls_back_to_corrected_e2e(self):
    raw = np.array([[0., 0.30, 0.], [10., 0.35, 0.]])
    lp = self.FakeLP(-0.04, -0.05, probability=0.0)
    e2e, lane = build_dnga_paths(raw, lp, 15., np.array([0., 1.]))
    np.testing.assert_allclose(e2e, lane)

  def test_path_offset_applied_once_on_repeated_calls(self):
    raw = np.array([[0., -0.03, 0.], [10., -0.02, 0.]])
    lp = self.FakeLP(-0.04, 0.05, probability=0.7)
    a, b = build_dnga_paths(raw, lp, 20., np.array([0., 1.]))
    c, d = build_dnga_paths(raw, lp, 20., np.array([0., 1.]))
    np.testing.assert_allclose(a, c)
    np.testing.assert_allclose(b, d)
    np.testing.assert_allclose(raw[:, 1], [-0.03, -0.02])

  def test_zero_offsets_keep_raw_e2e(self):
    raw = np.array([[0., 0.13, 0.], [20., 0.14, 0.]])
    lp = self.FakeLP(0., 0.)
    e2e, lane = build_dnga_paths(raw, lp, 25., np.array([0., 1.]))
    np.testing.assert_allclose(e2e, raw)
    self.assertGreater(np.max(np.abs(e2e[:, 1] - lane[:, 1])), 0.1)


if __name__ == "__main__":
  unittest.main()
