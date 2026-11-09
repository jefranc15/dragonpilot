import os

from common.params import Params

from selfdrive.hardware import EON, TICI, PC
from selfdrive.manager.process import PythonProcess, NativeProcess, DaemonProcess

WEBCAM = os.getenv("USE_WEBCAM") is not None
MIPI = os.getenv("USE_MIPI") is not None
USE_HYBRID_MODEL = Params().get_bool("NavSettingTime24h") and not os.path.isfile("/data/media/0/use_dp_0813_model")

procs = [
  DaemonProcess("manage_athenad", "selfdrive.athena.manage_athenad", "AthenadPid"),
  # due to qualcomm kernel bugs SIGKILLing camerad sometimes causes page table corruption
  NativeProcess("camerad", "selfdrive/camerad", ["./camerad"], unkillable=True, driverview=True),
  NativeProcess("clocksd", "selfdrive/clocksd", ["./clocksd"]),
  NativeProcess("dmonitoringmodeld", "selfdrive/modeld", ["./dmonitoringmodeld"], enabled=not MIPI and (not PC or WEBCAM), driverview=True),
  NativeProcess("logcatd", "selfdrive/logcatd", ["./logcatd"]),
  NativeProcess("loggerd", "selfdrive/loggerd", ["./loggerd"]),
  # Default to the R2/LegacyPilot 0.8.16 hybrid driving model. Setting
  # this marker restores the original DP 0.8.13 model without reinstalling.
  NativeProcess("modeld", "selfdrive/hybrid_modeld" if USE_HYBRID_MODEL else "selfdrive/modeld", ["sh", "./modeld"] if USE_HYBRID_MODEL else ["./modeld"]),
  NativeProcess("navd", "selfdrive/ui/navd", ["./navd"], persistent=True),
  NativeProcess("proclogd", "selfdrive/proclogd", ["./proclogd"]),
  NativeProcess("sensord", "selfdrive/sensord", ["./sensord"], enabled=not PC and not MIPI, persistent=EON, sigkill=EON),
  NativeProcess("ubloxd", "selfdrive/locationd", ["./ubloxd"], enabled=False),
  NativeProcess("ui", "selfdrive/ui", ["./ui"], persistent=True, watchdog_max_dt=(5 if TICI else None)),
  NativeProcess("soundd", "selfdrive/ui/soundd", ["./soundd"], persistent=True, enabled= not MIPI),
  NativeProcess("locationd", "selfdrive/locationd", ["./locationd"], persistent=True),
  NativeProcess("phonegpsd", "selfdrive/locationd", ["python", "phonegpsd.py"], enabled=False, persistent=False),
  NativeProcess("gpswaked", "selfdrive/locationd", ["python", "gpswaked.py"], enabled=False, persistent=False),
  NativeProcess("rotationlockd", "selfdrive/locationd", ["python", "rotationlockd.py"], enabled=False, persistent=False),
  NativeProcess("boardd", "selfdrive/boardd", ["./boardd"], enabled=False),
  PythonProcess("calibrationd", "selfdrive.locationd.calibrationd"),
  PythonProcess("controlsd", "selfdrive.controls.controlsd"),
  PythonProcess("deleter", "selfdrive.loggerd.deleter", persistent=True),
  PythonProcess("dmonitoringd", "selfdrive.monitoring.dmonitoringd", enabled=not MIPI and (not PC or WEBCAM), driverview=True),
  PythonProcess("logmessaged", "selfdrive.logmessaged", persistent=True),
  PythonProcess("pandad", "selfdrive.pandad", persistent=True),
  PythonProcess("paramsd", "selfdrive.locationd.paramsd"),
  PythonProcess("plannerd", "selfdrive.controls.plannerd"),
  PythonProcess("radard", "selfdrive.controls.radard"),
  PythonProcess("thermald", "selfdrive.thermald.thermald", persistent=True),
  PythonProcess("timezoned", "selfdrive.timezoned", enabled=TICI, persistent=True),
  PythonProcess("tombstoned", "selfdrive.tombstoned", enabled=not PC and not MIPI, persistent=True),
  PythonProcess("updated", "selfdrive.updated", enabled=not PC, persistent=True),
  PythonProcess("uploader", "selfdrive.loggerd.uploader", enabled=not MIPI, persistent=True),
  PythonProcess("statsd", "selfdrive.statsd", persistent=True),
  PythonProcess("mapd", "selfdrive.mapd.mapd"),

  # EON only
  PythonProcess("rtshield", "selfdrive.rtshield", enabled=EON),
  PythonProcess("shutdownd", "selfdrive.hardware.eon.shutdownd", enabled=EON),
  PythonProcess("androidd", "selfdrive.hardware.eon.androidd", enabled=EON, persistent=True),

  # dp
  PythonProcess("systemd", "selfdrive.dragonpilot.systemd", persistent=True),
  PythonProcess("gpxd", "selfdrive.dragonpilot.gpxd"),
  PythonProcess("otisserv", "selfdrive.dragonpilot.otisserv", persistent=True),
]

managed_processes = {p.name: p for p in procs}
