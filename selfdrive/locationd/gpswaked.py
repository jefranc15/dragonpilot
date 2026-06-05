#!/usr/bin/env python3
import subprocess
import time

ANDROID_ENV = {
  "PATH": "/sbin:/system/sbin:/system/bin:/system/xbin:/vendor/bin",
  "ANDROID_ROOT": "/system",
  "ANDROID_DATA": "/data",
}

PKG = "com.chartcross.gpstest"

def run(cmd, timeout=10):
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
  out = run(["/system/bin/ps", "-A"], timeout=5)
  return PKG in out

def gps_requested():
  out = run(["/system/bin/dumpsys", "location"], timeout=5)
  return "UpdateRecord[gps com.chartcross.gpstest" in out

def launch_gpstest():
  print("gpswaked: launching GPS Test", flush=True)
  print(run([
    "/system/bin/sh",
    "-c",
    "/system/bin/monkey -p com.chartcross.gpstest -c android.intent.category.LAUNCHER 1"
  ], timeout=15), flush=True)

  # Let GPS Test start its GPS request, then return to dragonpilot UI/Home.
  time.sleep(10)
  run([
    "/system/bin/sh",
    "-c",
    "/system/bin/input keyevent 3"
  ], timeout=5)

def main():
  time.sleep(2)

  while True:
    if not app_running() or not gps_requested():
      launch_gpstest()
      time.sleep(10)

    time.sleep(120)

if __name__ == "__main__":
  main()
