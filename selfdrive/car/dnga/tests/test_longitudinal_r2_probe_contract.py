"""Off-car R2 probe test with a fake V5.0 parent (no cereal import needed)."""
import unittest
from types import SimpleNamespace
from pathlib import Path

class StubV50:
  def __init__(self):
    self.brake_active = True
    self.curve_brake_active = True
    self.handoff_active = True
    self.handoff_counter = 3
    self.release_counter = 10
    self.coast_release_counter = 4
    self.lead_loss_counter = 0
    self.apply_brake = .05
    self.sng_armed = False
    self.stop_hold = False
    self.urgent_brake = False
    self.brake_reentry_frame = 17

  def _clear_hydraulic(self, frame, reentry=True):
    self.brake_active = False
    self.release_counter = 0
    self.handoff_active = False
    return 'UNCHANGED_RESULT'

  def _update_hydraulic(self, CS, frame, session, apply_accel, plan, lead_state):
    self._clear_hydraulic(frame)
    return 'UNCHANGED_COMMAND'

  def _update_stop_guard(self, *args):
    return SimpleNamespace(active=False, entry=False, predictive_confirmed=False, completion=False)


src = (Path(__file__).resolve().parents[1]/'longitudinal_r2_instrumented.py').read_text()
src = src.replace(
  'from selfdrive.car.dnga.longitudinal_kinematic_guard import LongitudinalController as V50Controller\n', ''
)
ns = {'V50Controller': StubV50}
exec(compile(src, 'longitudinal_r2_instrumented.py', 'exec'), ns)
Probe = ns['LongitudinalController']


class TestR2Probe(unittest.TestCase):
  def test_preserves_command_and_captures_pre_reset_state(self):
    p = Probe()
    cs = SimpleNamespace(out=SimpleNamespace(vEgo=20., aEgo=-.2))
    session = SimpleNamespace(allowed=True)
    plan = SimpleNamespace(fresh=True, accel=-.15, brake=.15, curve_active=False)
    lead = SimpleNamespace(relevant=True, status=True, closing_speed=2.,
                           distance=25., ttc=12.5, urgent=False)

    result = p._update_hydraulic(cs, 20, session, .8, plan, lead)
    self.assertEqual(result, 'UNCHANGED_COMMAND')
    self.assertFalse(p.brake_active)
    self.assertEqual([e['event'] for e in p.r2_events], ['hydraulic_clear'])
    e = p.r2_events[0]
    self.assertEqual(e['release_counter'], 10)
    self.assertTrue(e['handoff_active'])
    self.assertEqual(e['coast_release_counter'], 4)
    self.assertTrue(e['curve_brake_active'])
    self.assertTrue(e['context']['curve_brake_active'])
    self.assertAlmostEqual(e['context']['planner_brake'], .15)
    self.assertAlmostEqual(e['context']['pid_accel'], .8)


if __name__ == '__main__':
  unittest.main()
