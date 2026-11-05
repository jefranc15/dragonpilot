"""Pure off-car regression tests for V5.0-R1 experimental policy."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from longitudinal_r1_policy import (
  CoastPreparationLatch, LeadBlipGrace, should_prepare_coast, should_bridge_lead_blip,
)

COAST = dict(allowed=True, moving_speed=8.1, plan_fresh=True,
             lead_relevant=True, lead_status=True, distance=39.0,
             closing=4.5, ttc=8.7, urgent=False, curve=False,
             hydraulic=False, release=False, hold=False)
BRIDGE = dict(allowed=True, braking=True, curve=False, plan_fresh=True,
              plan_has_lead=False, radar_fresh=True, lead_status=True,
              distance=28.0, closing=2.0, ttc=14.0, planner_brake=0.20)


class TestR1Policy(unittest.TestCase):
  def test_coast_preparation_debounce(self):
    latch = CoastPreparationLatch()
    self.assertTrue(should_prepare_coast(**COAST))
    self.assertFalse(latch.update(True))
    self.assertFalse(latch.update(True))
    self.assertTrue(latch.update(True))
    self.assertTrue(latch.update(True))
    self.assertFalse(latch.update(False))

  def test_coast_never_arms_for_untrusted_or_unsafe_context(self):
    for name,value in [('allowed',False),('plan_fresh',False),('lead_relevant',False),
                       ('lead_status',False),('urgent',True),('curve',True),
                       ('hydraulic',True),('release',True),('hold',True),
                       ('closing',0.0),('ttc',40.0),('moving_speed',4.0),
                       ('distance',150.0)]:
      with self.subTest(name=name):
        self.assertFalse(should_prepare_coast(**{**COAST,name:value}))

  def test_brake_grace_maintains_only_two_valid_dropouts(self):
    self.assertTrue(should_bridge_lead_blip(**BRIDGE))
    grace = LeadBlipGrace()
    self.assertFalse(grace.update(braking=True,plan_has_lead=True,
                                  trusted_lead=True,eligible=False))
    self.assertTrue(grace.update(braking=True,plan_has_lead=False,
                                 trusted_lead=False,eligible=True))
    self.assertTrue(grace.update(braking=True,plan_has_lead=False,
                                 trusted_lead=False,eligible=True))
    self.assertFalse(grace.update(braking=True,plan_has_lead=False,
                                  trusted_lead=False,eligible=True))
    self.assertFalse(grace.update(braking=True,plan_has_lead=True,
                                  trusted_lead=False,eligible=False))
    self.assertFalse(grace.update(braking=True,plan_has_lead=False,
                                  trusted_lead=False,eligible=True))

  def test_brake_grace_immediate_loss_for_unsafe_inputs(self):
    for name,value in [('allowed',False),('curve',True),('radar_fresh',False),
                       ('lead_status',False),('closing',0.0),('ttc',30.0),
                       ('distance',95.0),('planner_brake',0.0),('plan_fresh',False)]:
      with self.subTest(name=name):
        self.assertFalse(should_bridge_lead_blip(**{**BRIDGE,name:value}))
    grace=LeadBlipGrace()
    grace.update(braking=True,plan_has_lead=True,trusted_lead=True,eligible=False)
    self.assertFalse(grace.update(braking=False,plan_has_lead=False,
                                  trusted_lead=False,eligible=True))
    self.assertFalse(grace.update(braking=True,plan_has_lead=False,
                                  trusted_lead=False,eligible=True))

  def test_brake_grace_requires_missing_planner_lead(self):
    self.assertFalse(should_bridge_lead_blip(**{**BRIDGE,'plan_has_lead':True}))

if __name__ == '__main__':
  unittest.main()
