"""Read-only R2 candidate diagnostics. No vehicle control or CAN output."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from v60_offline_turn_lead_review import lead_response_timeline, turn_advisory


class OfflineReplayTest(unittest.TestCase):
  def test_missed_curve_emits_one_research_advisory(self):
    # Sparse 20 Hz selected-source updates matching Nov 12 missed turn.
    rows = [
      dict(t=2544.855, v_kph=73.77, vision_target_kph=71.8, source="turn", has_lead=False, p_amin=-.054),
      dict(t=2544.906, v_kph=73.75, vision_target_kph=0, source="cruise", has_lead=False, p_amin=-.06),
      dict(t=2545.019, v_kph=73.63, vision_target_kph=71.28, source="turn", has_lead=False, p_amin=-.0753),
      dict(t=2545.073, v_kph=73.65, vision_target_kph=0, source="cruise", has_lead=False, p_amin=-.09),
      dict(t=2545.124, v_kph=73.68, vision_target_kph=70.9, source="turn", has_lead=False, p_amin=-.095),
      dict(t=2545.325, v_kph=73.64, vision_target_kph=70.74, source="turn", has_lead=False, p_amin=-.102),
    ]
    events = turn_advisory(rows)
    self.assertEqual(len(events), 1)
    self.assertEqual(events[0]["time_s"], 2545.124)
    self.assertEqual(events[0]["kind"], "READ_ONLY_TURN_ADVISORY")

  def test_old_35_kph_curve_chatter_does_not_trigger_high_speed_hypothesis(self):
    rows = [
      dict(t=405 + .05*i, v_kph=35.4, vision_target_kph=31.0,
           source="turn", p_amin=-.20, has_lead=False) for i in range(20)
    ]
    self.assertEqual(turn_advisory(rows), [])

  def test_lead_turn_source_never_authorizes_turn_only_advisory(self):
    rows = [
      dict(t=.05*i, v_kph=73, vision_target_kph=67,
           source="turn", p_amin=-.3, has_lead=True) for i in range(10)
    ]
    self.assertEqual(turn_advisory(rows), [])

  def test_single_source_flicker_cannot_trigger(self):
    rows = [
      dict(t=.05*i, v_kph=74, vision_target_kph=68,
           source="turn" if i == 2 else "cruise",
           p_amin=-.2, has_lead=False) for i in range(10)
    ]
    self.assertEqual(turn_advisory(rows), [])

  def test_driver_override_cannot_trigger(self):
    rows = [
      dict(t=.05*i, v_kph=74, vision_target_kph=68,
           source="turn", p_amin=-.2, has_lead=False, driver_brake=True)
      for i in range(10)
    ]
    self.assertEqual(turn_advisory(rows), [])

  def test_positive_future_plan_cannot_trigger(self):
    rows = [
      dict(t=.05*i, v_kph=74, vision_target_kph=68,
           source="turn", p_amin=.02, has_lead=False) for i in range(10)
    ]
    self.assertEqual(turn_advisory(rows), [])

  def test_timeline_is_observational_not_regen_calibration(self):
    rows = [
      dict(t=1.0, lead_status=True, p_amin=.02, brake_state=1),
      dict(t=1.5, lead_status=True, p_amin=-.06, brake_state=1),
      dict(t=1.8, lead_status=True, p_amin=-.10, brake_state=33, brake_norm=.08, a=0.),
      dict(t=3.0, lead_status=True, p_amin=-.6, brake_state=33, brake_norm=.20, a=-.6),
    ]
    result = lead_response_timeline(rows)
    self.assertEqual(result["sent_strength_ge_0_20_after_first_brake_s"], 1.2)
    self.assertIn("regen/friction", result["interpretation"])


if __name__ == "__main__":
  unittest.main()
