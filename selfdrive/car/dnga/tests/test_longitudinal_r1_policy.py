"""Pure off-car regression tests for V5.0-R1 experimental policy."""
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from longitudinal_r1_policy import CoastPreparationLatch, should_prepare_coast, should_abort_handoff

COAST=dict(allowed=True,moving_speed=8.1,plan_fresh=True,
           lead_relevant=True,lead_status=True,distance=39.,closing=4.5,
           ttc=8.7,urgent=False,curve=False,hydraulic=False,release=False,hold=False)
HANDOFF=dict(allowed=True,braking=True,curve=False,hold=False,
             plan_fresh=True,lead_relevant=True,lead_status=True,
             planner_brake=.2,pid_accel=-1.0)


class TestR1Policy(unittest.TestCase):
  def test_coast_preparation_debounce(self):
    latch=CoastPreparationLatch()
    self.assertTrue(should_prepare_coast(**COAST))
    self.assertFalse(latch.update(True))
    self.assertFalse(latch.update(True))
    self.assertTrue(latch.update(True))
    self.assertTrue(latch.update(True))
    self.assertFalse(latch.update(False))

  def test_coast_never_arms_for_untrusted_or_unsafe_context(self):
    for k,val in [('allowed',False),('plan_fresh',False),('lead_relevant',False),
                  ('lead_status',False),('urgent',True),('curve',True),
                  ('hydraulic',True),('release',True),('hold',True),
                  ('closing',0.),('ttc',40.),('moving_speed',4.),('distance',150.)]:
      with self.subTest(k=k):
        self.assertFalse(should_prepare_coast(**{**COAST,k:val}))

  def test_abort_old_handoff_when_demand_returns(self):
    self.assertTrue(should_abort_handoff(**HANDOFF))
    self.assertFalse(should_abort_handoff(**{**HANDOFF,'pid_accel':.3}))
    self.assertFalse(should_abort_handoff(**{**HANDOFF,'planner_brake':.01}))

  def test_no_abort_when_untrusted_or_driver_override(self):
    for k,val in [('allowed',False),('braking',False),('curve',True),
                  ('hold',True),('plan_fresh',False),('lead_relevant',False),
                  ('lead_status',False)]:
      with self.subTest(k=k):
        self.assertFalse(should_abort_handoff(**{**HANDOFF,k:val}))

if __name__=='__main__':unittest.main()
