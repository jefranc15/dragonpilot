#!/usr/bin/env python3
"""Driver-monitoring stub for the OnePlus 3T night-test build.

The DIY OnePlus 3T setup has no IR illuminator and retains the camera IR filter,
so the normal driver-monitoring model is not useful at night. Keep the
`driverMonitoringState` service alive and valid, but publish no distraction
alerts. This mirrors the practical no-DM behavior used on the LG G8 AGNOS
setup without changing any longitudinal or lateral control logic.
"""

import cereal.messaging as messaging
from common.realtime import Ratekeeper


def main():
  pm = messaging.PubMaster(["driverMonitoringState"])
  rk = Ratekeeper(10, print_delay_threshold=None)

  while True:
    msg = messaging.new_message("driverMonitoringState")
    msg.driverMonitoringState.events = []
    msg.driverMonitoringState.faceDetected = True
    msg.driverMonitoringState.isDistracted = False
    msg.driverMonitoringState.awarenessStatus = 1.0
    msg.driverMonitoringState.awarenessActive = 1.0
    msg.driverMonitoringState.awarenessPassive = 1.0
    msg.driverMonitoringState.isActiveMode = False
    pm.send("driverMonitoringState", msg)
    rk.keep_time()


if __name__ == "__main__":
  main()
