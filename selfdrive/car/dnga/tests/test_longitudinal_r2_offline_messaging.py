"""Offline replay bootstrap regression; no pycapnp or compiled messaging_pyx needed."""
import importlib
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch


def load_bootstrap():
  src = (Path(__file__).resolve().parent / 'r2_replay_offline_messaging.py').read_text()
  ns = {'__name__': '_r2_offline_bootstrap_test_'}
  exec(compile(src, '<r2_replay_offline_messaging>', 'exec'), ns)
  return ns['install_offline_messaging']


class TestR2OfflineMessaging(unittest.TestCase):
  def test_offline_controller_import_dependency_is_stubbed(self):
    # Simulate cereal installed without cereal.messaging.messaging_pyx.
    # Real controllers never need network sockets in this replay.
    installer = load_bootstrap()
    cereal = types.ModuleType('cereal')
    cereal.__path__ = []
    with patch.dict(sys.modules, {'cereal': cereal}):
      sys.modules.pop('cereal.messaging', None)
      installer()
      from cereal import messaging
      self.assertIs(messaging, sys.modules['cereal.messaging'])
      self.assertIs(messaging, cereal.messaging)
      with self.assertRaisesRegex(RuntimeError, 'offline replay'):
        messaging.SubMaster(['longitudinalPlan'])
      with self.assertRaisesRegex(RuntimeError, 'before importing'):
        installer()

  def test_existing_native_messaging_is_not_overwritten(self):
    installer = load_bootstrap()
    cereal = types.ModuleType('cereal')
    existing = types.ModuleType('cereal.messaging')
    with patch.dict(sys.modules, {'cereal': cereal, 'cereal.messaging': existing}):
      with self.assertRaisesRegex(RuntimeError, 'before importing'):
        installer()
      self.assertIs(sys.modules['cereal.messaging'], existing)


if __name__ == '__main__':
  unittest.main()
