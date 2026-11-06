"""Read-only tests for the offline R2/V4 replay audit."""
import csv
import os
import tempfile
import unittest
from r2_v4_replay_audit_readonly import audit

COLUMNS = ['original_state', 'v4_state', 'bridge', 'unserved_negative_demand',
           'calibration_outside_stock', 'feedback_stale', 'v4_is_accel',
           'hybrid_brake_raw', 'friction_raw']

class TestAudit(unittest.TestCase):
  def test_counterfactual_disagreements_and_feedback(self):
    rows = [
      dict(original_state='1',v4_state='33',bridge='True',
           unserved_negative_demand='False',calibration_outside_stock='True',
           feedback_stale='False',v4_is_accel='False',
           hybrid_brake_raw='-200',friction_raw='0'),
      dict(original_state='33',v4_state='1',bridge='False',
           unserved_negative_demand='True',calibration_outside_stock='False',
           feedback_stale='False',v4_is_accel='True',
           hybrid_brake_raw='0',friction_raw='0'),
      dict(original_state='1',v4_state='1',bridge='False',
           unserved_negative_demand='False',calibration_outside_stock='False',
           feedback_stale='True',v4_is_accel='False',
           hybrid_brake_raw='',friction_raw=''),
    ]
    with tempfile.NamedTemporaryFile(mode='w',newline='',suffix='.csv',
                                     delete=False) as file:
      path = file.name
      writer = csv.DictWriter(file,fieldnames=COLUMNS)
      writer.writeheader()
      writer.writerows(rows)
    try:
      output = audit(path)
      self.assertEqual(output['frames'], 3)
      self.assertEqual(output['new_braking_relative_to_v50_ready'], 1)
      self.assertEqual(output['missing_braking_relative_to_v50'], 1)
      self.assertEqual(output['source_bridge_frames'], 1)
      self.assertEqual(output['outside_stock_calibration'], 1)
      self.assertEqual(output.get('accel_with_recorded_brake_feedback', 0), 0)
    finally:
      os.unlink(path)

if __name__ == '__main__':
  unittest.main()
