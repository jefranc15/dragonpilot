#!/usr/bin/env python3
"""DNGA fixed vehicle-model publisher.

The Yaris Cross DNGA port does not use locationd/paramsd learning for lateral
control. Camera and driving-path offsets remain live through dragonConf in the
lateral planner. This publisher keeps the legacy liveParameters topic healthy
for controlsd while using fixed CarParams values only.
"""

import time

from cereal import car
import cereal.messaging as messaging
from common.params import Params


PUBLISH_DIVIDER = 5  # carState is 100 Hz; publish liveParameters at ~20 Hz


def _set_dnga_static_params(params, steer_ratio):
  params.put("dp_sr_learner", "0")
  params.put("dp_sr_custom", "{:.2f}".format(steer_ratio))
  params.put("dp_sr_stock", "{:.2f}".format(steer_ratio))
  params.put("dp_last_modified", str(int(time.time())))


def static_dnga_paramsd(CP, sm=None, pm=None):
  params = Params()
  fixed_steer_ratio = float(CP.steerRatio)
  _set_dnga_static_params(params, fixed_steer_ratio)

  if sm is None:
    sm = messaging.SubMaster(["carState"], poll=["carState"])
  if pm is None:
    pm = messaging.PubMaster(["liveParameters"])

  frame = 0
  while True:
    sm.update()
    if not sm.updated["carState"]:
      continue

    frame += 1
    if frame % PUBLISH_DIVIDER:
      continue

    cs = sm["carState"]
    msg = messaging.new_message("liveParameters")
    msg.logMonoTime = sm.logMonoTime["carState"]

    lp = msg.liveParameters
    lp.valid = True
    lp.sensorValid = True
    lp.posenetValid = True

    lp.gyroBias = 0.0
    lp.angleOffsetDeg = 0.0
    lp.angleOffsetAverageDeg = 0.0
    lp.stiffnessFactor = 1.0
    lp.steerRatio = fixed_steer_ratio
    lp.yawRate = float(getattr(cs, "yawRate", 0.0))
    lp.posenetSpeed = float(cs.vEgo)
    lp.roll = 0.0

    lp.angleOffsetFastStd = 0.0
    lp.angleOffsetAverageStd = 0.0
    lp.stiffnessFactorStd = 0.0
    lp.steerRatioStd = 0.0

    pm.send("liveParameters", msg)


def main(sm=None, pm=None):
  params = Params()
  CP = car.CarParams.from_bytes(params.get("CarParams", block=True))

  if CP.carName != "dnga":
    # Preserve the stock learner for every non-DNGA port on this branch.
    from selfdrive.locationd.paramsd import main as dynamic_main
    return dynamic_main(sm=sm, pm=pm)

  return static_dnga_paramsd(CP, sm=sm, pm=pm)


if __name__ == "__main__":
  main()
