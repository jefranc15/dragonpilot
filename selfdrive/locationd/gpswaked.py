#!/usr/bin/env python3
import subprocess
import time

ANDROID_ENV = {
  "PATH": "/sbin:/system/sbin:/system/bin:/system/xbin:/vendor/bin",
  "ANDROID_ROOT": "/system",
  "ANDROID_DATA": "/data",
}

PKG = "com.chartcross.gpstest"

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

def gps_requested():
  out = run(["/system/bin/dumpsys", "location"], timeout=3)
  return "UpdateRecord[gps com.chartcross.gpstest" in out

def launch_gpstest():
  print("gpswaked: launching GPS Test", flush=True)
  print(run([
    "/system/bin/sh",
    "-c",
    "/system/bin/monkey -p com.chartcross.gpstest -c android.intent.category.LAUNCHER 1"
  ], timeout=3), flush=True)

  # Let GPS Test start its GPS request, then hide it.
  time.sleep(1)
  print("gpswaked: pressing Back", flush=True)
  print(run([
    "/system/bin/sh",
    "-c",
    "/system/bin/input keyevent 4"
  ], timeout=3), flush=True)

def main():
  time.sleep(20)

  while True:
    if not gps_requested():
      launch_gpstest()

    # Do not relaunch periodically if GPS is already active.
    time.sleep(300)

if __name__ == "__main__":
  main()
