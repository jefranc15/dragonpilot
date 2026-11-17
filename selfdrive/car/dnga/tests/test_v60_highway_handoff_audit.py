"""Offline-only HEV release anomaly detector tests; no ECU or Panda required."""
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from v60_highway_handoff_audit import anomalies, ready_releases, audit


class HighwayHandoffAuditTests(unittest.TestCase):
  def row(self, t, **update):
    data=dict(t=t,v_kph=66,a=0,rpm=1000,brake_state=1,pump=0,
              enabled_y=True,pedal=0,driver_brake=False,is_accel=False,
              is_decel=False,accel=0,p_a0=0,set_speed=75,
              camera_enabled=False,camera_is_accel=False,camera_is_decel=False)
    data.update(update)
    return data

  def test_detects_opposite_pid_neutral_launch(self):
    rows=[self.row(i*.05,a=1.5,accel=-1.5,rpm=3300) for i in range(20)]
    self.assertEqual([x['kind'] for x in anomalies(rows)],
                     ['NEUTRAL_POSITIVE_ACCEL_NEGATIVE_PID'])

  def test_driver_pedal_override_excluded(self):
    rows=[self.row(i*.05,a=1.5,accel=-1.5,rpm=3300,pedal=4000) for i in range(20)]
    self.assertEqual(anomalies(rows),[])

  def test_positive_op_mode_not_neutral(self):
    rows=[self.row(i*.05,a=1.5,accel=-1.5,rpm=3300,is_accel=True) for i in range(20)]
    self.assertEqual(anomalies(rows),[])

  def test_detects_negative_ae_with_positive_pid(self):
    rows=[self.row(i*.05,a=-.7,accel=2,rpm=3200) for i in range(20)]
    self.assertEqual(anomalies(rows)[0]['kind'],'NEUTRAL_DECEL_POSITIVE_PID')

  def test_camera_difference_is_only_an_intent_warning(self):
    rows=[self.row(i*.05,camera_enabled=True,camera_is_accel=True,
                   is_decel=True) for i in range(20)]
    self.assertEqual(anomalies(rows)[0]['kind'],'CAMERA_INTENT_OP_DECEL_CONFLICT')
    self.assertIn('does not prove',audit(rows)['limitations'])

  def test_sustained_brake_pid_disagreement(self):
    rows=[self.row(i*.05,brake_state=33,accel=1,rpm=3200,p_a0=-.3) for i in range(30)]
    self.assertEqual(anomalies(rows)[0]['kind'],'BRAKING_POSITIVE_PID_HIGH_RPM')

  def test_ignores_transient_one_sample_noise(self):
    rows=[self.row(i*.05,a=1.5,accel=-1) if i==4 else self.row(i*.05) for i in range(30)]
    self.assertEqual(anomalies(rows),[])

  def test_ready_release_measures_actual_response(self):
    rows=[self.row(0,brake_state=33),self.row(.05,pump=-.4)]
    rows.extend(self.row(.1+i*.05,a=1.5,rpm=3500) for i in range(20))
    result=ready_releases(rows)
    self.assertEqual(len(result),1)
    self.assertEqual(result[0]['peak_aego_next_4s'],1.5)

  def test_disabled_transition_not_counted_as_ready_release(self):
    rows=[self.row(0,brake_state=33),self.row(.05,brake_state=0)]
    rows.extend(self.row(.1+i*.05,a=1.5) for i in range(20))
    self.assertEqual(ready_releases(rows),[])

  def test_hold_does_not_count_as_neutral_launch(self):
    rows=[self.row(i*.05,brake_state=48,a=1.5,accel=-1) for i in range(20)]
    self.assertEqual(anomalies(rows),[])


if __name__=='__main__':
  unittest.main()
