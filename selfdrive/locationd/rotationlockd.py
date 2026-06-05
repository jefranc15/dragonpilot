#!/usr/bin/env python3
import subprocess
import time

CMD = (
  "settings put system accelerometer_rotation 0; "
  "settings put system user_rotation 1; "
  "settings put secure accelerometer_rotation 0; "
  "settings put secure user_rotation 1; "
  "wm user-rotation lock 1 2>/dev/null || true; "
  "cmd window set-user-rotation lock 1 2>/dev/null || true"
)

ENV = {
  "PATH": "/sbin:/system/sbin:/system/bin:/system/xbin:/vendor/bin",
  "ANDROID_ROOT": "/system",
  "ANDROID_DATA": "/data",
}

while True:
  subprocess.call(["/system/bin/sh", "-c", CMD], env=ENV)
  time.sleep(2)
