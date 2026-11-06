"""Replay-only replacement for the native cereal.messaging extension.

Openpilot's longitudinal.py imports cereal.messaging solely to initialize
SubMaster. Offline rlog replay replaces BOTH SubMasters with RecordedFeed,
so it must never connect to the live messaging bus. The compiled messaging_pyx
module is not present in a clean Git worktree on NEOS/Termux.

Only import this from the standalone replay entrypoint, NEVER from production
CarController or vehicle code.
"""
import sys
from types import ModuleType


def install_offline_messaging():
  # Fail closed if the actual messaging API was already loaded, avoiding
  # silently altering any running service or mixed-import environment.
  if 'cereal.messaging' in sys.modules:
    raise RuntimeError('Offline replay must bootstrap before importing cereal.messaging')

  import cereal

  def reject_live_submaster(*_args, **_kwargs):
    raise RuntimeError('Live messaging is disabled in offline replay; use RecordedFeed')

  offline = ModuleType('cereal.messaging')
  offline.SubMaster = reject_live_submaster
  sys.modules['cereal.messaging'] = offline
  cereal.messaging = offline
