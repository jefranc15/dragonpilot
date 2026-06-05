#!/usr/bin/env python3
import subprocess
import time

from cereal import messaging

ANDROID_ENV = {
  "PATH": "/sbin:/system/sbin:/system/bin:/system/xbin:/vendor/bin",
  "ANDROID_ROOT": "/system",
  "ANDROID_DATA": "/data",
}

PKG = "com.chartcross.gpstest"

# Only allow GPS Test auto-launch shortly after boot/start.
# After this, it will never pop up while driving.
STARTUP_LAUNCH_WINDOW = 180  # seconds

def run(cmd, timeout=3):
  try:
    return subprocess.check_output(
      cmd,
      stderr=subprocess.STDOUT,
      text=True,
      errors="ignore",
      timeout=timeout,
      env=ANDROID_ENV,
    )
  except Exception as e:
    return str(e)

def app_running():
  out = run(["/system/bin/ps", "-A"], timeout=3)
  return PKG in out

def app_foreground():
  out = run([
    "/system/bin/sh",
    "-c",
    "dumpsys window windows | grep -iE 'mCurrentFocus|mFocusedApp'"
  ], timeout=3)
  return PKG in out

def gps_requested():
  out = run(["/system/bin/dumpsys", "location"], timeout=3)
  return "UpdateRecord[gps com.chartcross.gpstest" in out

def press_back():
  print("gpswaked: pressing Back", flush=True)
  print(run([
    "/system/bin/sh",
    "-c",
    "/system/bin/input keyevent 4"
  ], timeout=3), flush=True)

def launch_gpstest():
  print("gpswaked: launching GPS Test", flush=True)
  print(run([
    "/system/bin/sh",
    "-c",
    "/system/bin/monkey -p com.chartcross.gpstest -c android.intent.category.LAUNCHER 1"
  ], timeout=3), flush=True)

  time.sleep(1)
  press_back()

def main():
  start_time = time.monotonic()

  sm = messaging.SubMaster(["carState", "controlsState"])

  time.sleep(20)

  while True:
    sm.update(100)

    v_ego = getattr(sm["carState"], "vEgo", 0.0)
    controls_enabled = bool(getattr(sm["controlsState"], "enabled", False))

    moving = v_ego > 0.3
    driving_or_engaged = moving or controls_enabled

    # If GPS Test is ever foreground while driving, hide it.
    if app_foreground():
      press_back()

    within_startup_window = (time.monotonic() - start_time) < STARTUP_LAUNCH_WINDOW

    # Only auto-launch shortly after boot, and never while moving/engaged.
    if within_startup_window and not driving_or_engaged:
      if (not app_running()) or (not gps_requested()):
        launch_gpstest()

    time.sleep(30)

if __name__ == "__main__":
  main()
