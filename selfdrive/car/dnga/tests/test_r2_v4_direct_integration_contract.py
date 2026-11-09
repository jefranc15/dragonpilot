"""Source-level integration contract for bench-only R2 direct moving-brake branch.

The device's compiled cereal/DBC runtime is unavailable in cloud CI, so these
tests validate the CarController integration structure without claiming HIL.
"""
from pathlib import Path
import ast
import unittest


CONTROLLER = Path(__file__).resolve().parents[1] / "carcontroller.py"


class DirectIntegrationContractTests(unittest.TestCase):
  def setUp(self):
    self.source = CONTROLLER.read_text()
    self.tree = ast.parse(self.source)

  def test_default_is_non_actuating(self):
    values = {n.targets[0].id: n.value.value
              for n in self.tree.body if isinstance(n, ast.Assign)
              and len(n.targets) == 1 and isinstance(n.targets[0], ast.Name)
              and isinstance(n.value, ast.Constant)}
    self.assertIs(values["R2_V4_DIRECT_CAN_ENABLED"], False)

  def test_only_direct_r2_authority_is_selected(self):
    self.assertIn("self.r2_v4_direct.apply(", self.source)
    self.assertNotIn("self.r2_v4_actuator.apply(", self.source)
    self.assertNotIn("R2V4ExperimentalActuator", self.source)

  def test_r2_final_interlock_runs_after_direct_selection(self):
    i = self.source.index("command, r2_status = self.r2_v4_direct.apply(")
    j = self.source.index("command = self._apply_r2_final_brake_interlock(")
    k = self.source.index("dnga_create_accel_command(", j)
    self.assertLess(i, j)
    self.assertLess(j, k)

  def test_0x273_target_not_neutralized_in_direct_interlock(self):
    start = self.source.index("  def _apply_r2_final_brake_interlock(")
    end = self.source.index("  def _observe_r2_v4(", start)
    interlock = self.source[start:end]
    self.assertNotIn("command.speed = float(CS.out.vEgo)", interlock.split(
      "elif braking_feedback or not fresh:")[0])
    self.assertIn("command.is_accel = False", interlock)


if __name__ == "__main__":
  unittest.main()
