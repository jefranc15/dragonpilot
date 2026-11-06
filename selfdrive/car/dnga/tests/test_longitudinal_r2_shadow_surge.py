"""Unit tests for offline R2 no-lead surge screen. No vehicle imports."""
import csv
import tempfile
import unittest
from pathlib import Path

from r2_shadow_surge_screen import find_episodes, surge_candidate


def example(**changes):
  row = dict(route='test', t_ns='1000000000', frame='100', brake_state='1',
             enabled='True', lead='True', plan_has_lead='False',
             pedal_gas='False', pedal_brake='False', is_accel='False',
             a_ego='1.2', pid_accel='-2.0', v_ego='22.0',
             plan_source='turn')
  row.update(changes)
  return row


class TestR2SurgeScreen(unittest.TestCase):
  def test_hud_lead_does_not_override_no_planner_lead(self):
    self.assertTrue(surge_candidate(example(lead='True')))
    self.assertFalse(surge_candidate(example(plan_has_lead='True')))

  def test_no_driver_override_or_braking_state(self):
    for change in ({'pedal_gas': 'True'}, {'pedal_brake': 'True'},
                   {'is_accel': 'True'}, {'brake_state': '33'},
                   {'pid_accel': '0.0'}, {'a_ego': '0.1'},
                   {'enabled': 'False'}):
      with self.subTest(change=change):
        self.assertFalse(surge_candidate(example(**change)))

  def test_groups_only_contiguous_matching_frames(self):
    with tempfile.TemporaryDirectory() as directory:
      p = Path(directory) / 'test_trace.csv'
      frames = [example(t_ns='1000000000'),
                example(t_ns='1050000000'),
                example(t_ns='1100000000', a_ego='0.1'),
                example(t_ns='1150000000')]
      with p.open('w', newline='') as out:
        writer = csv.DictWriter(out, fieldnames=list(frames[0]))
        writer.writeheader()
        writer.writerows(frames)
      events = find_episodes([p])
      self.assertEqual(len(events), 2)
      self.assertEqual([e['n'] for e in events], [2, 1])


if __name__ == '__main__':
  unittest.main()
