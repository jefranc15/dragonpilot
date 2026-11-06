"""Startup smoke test for native 0.8.13 replay imports, without any CAN I/O."""
import os
import subprocess
import sys
import unittest
from pathlib import Path


class TestR2ReplayRuntimeImport(unittest.TestCase):
  def test_real_v50_controller_imports_without_messaging_pyx(self):
    # The same entrypoint order is used by r2_exact_controller_replay.py.
    # Running in a child process prevents module stubs leaking into tests.
    root = Path(__file__).resolve().parents[4]
    test_dir = Path(__file__).resolve().parent
    code = """
from cereal import log
from r2_replay_offline_messaging import install_offline_messaging
install_offline_messaging()
from selfdrive.car.dnga.longitudinal_kinematic_guard import LongitudinalController
from selfdrive.car.dnga.longitudinal_r2_instrumented import LongitudinalController as Probe
baseline = LongitudinalController()
instrumented = Probe()
assert baseline.plan_sm is None and baseline.radar_sm is None
assert instrumented.plan_sm is None and instrumented.radar_sm is None
print('R2 offline controller import OK')
"""
    env = os.environ.copy()
    env['PYTHONPATH'] = os.pathsep.join([str(test_dir), str(root), env.get('PYTHONPATH', '')])
    check = subprocess.run([sys.executable, '-c', code],
                           cwd=str(root), env=env,
                           capture_output=True, text=True)
    self.assertEqual(check.returncode, 0,
                     'Native replay startup import failed:\n' + check.stdout + check.stderr)


if __name__ == '__main__':
  unittest.main()
