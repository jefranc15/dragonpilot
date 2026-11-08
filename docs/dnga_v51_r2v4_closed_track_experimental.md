# R2 V4 experimental on-vehicle CAN command integration

**Branch:** `v5.1-R2V4-closed-track` (fork: jefranc15/dragonpilot)  
**Status:** ACTIVELY MODIFIES `0x271` and `0x273` when OP longitudinal is engaged.
**No hardware-in-the-loop results, no calibrated motor current/torque, and
no device-level test results have been verified for this exact commit.**
This code is experimental and not qualified for routine driving.

## Actual vs passive behavior

- `OpenpilotEnabledToggle=0` (Dashcam Mode): existing `controlsd`
  read-only/noOutput pathway remains unchanged. The factory camera
  controls stock ACC, with rlog recording if loggerd is enabled.
- OpenPilot enabled but **not engaged**: the R2 actuator resets; the existing
  V5.0 inactive-command path remains authoritative.
- OpenPilot enabled and engaged: compute the V5.0 command, apply its existing
  brake-propulsion interlock, compute the R2 V4 proposal from planner + PID,
  then optionally replace the actual moving BRAKING/READY command *before
  `dnga_create_accel_command` and `dnga_create_brake_command`*.
- Brake/CANCEL/gas override: no R2 override. The existing controlsd
  disengagement and Panda firmware are unchanged. The code does NOT
  independently establish that the physical override stops every ECU
  response. The current custom Panda safety hook is permissive, with
  `controls_allowed=true` and no RX-check list.
- Existing CREEPING/HOLD and initial low-speed control stay V5.0. Once
  R2 moving braking has started, it can continue across the 5.6 m/s
  boundary until the ordinary release process, to prevent a blind handoff.

## Implemented experimental behavior

1. R2 V4 planning/preview/source-bridge observer from
   `r2_v4_shadow_controller.py`, unchanged from the preceding shadow branch.
2. When active, the new `r2_v4_experimental_actuator.py` submits real
   `0x271 BRAKING`, using `MAGNITUDE=1216` / pump `-0.4` for mild entry,
   with strength rising in nominal 0.02 per-20-Hz steps; urgent
   steps nominal 0.05. Existing stronger V5.0 braking is not weakened.
3. During active 0x271 braking, it submits a below-speed `0x273 ACC_CMD`
   following the 0.18 km/h-per-50-ms target-gap slew calculated by the V4
   observer, with IS_DECEL=true and IS_ACCEL=false. This deliberately
   removes V5.0's unconditional neutral-0x273 override in those frames.
4. BRAKING -> READY with `MAGNITUDE=1224` / pump `-0.4` modeled from
   observed stock ACC release frames. The target offset returns gradually
   and positive ACCEL remains vetoed while release is active.
5. The module records `R2V4_SHADOW_ONLY` comparison messages and
   `R2V4_EXPERIMENTAL_TX` events with actual and base 0x271 state/magnitude,
   0x273 requested speed, decoded 0x275 and 0x08C feedback, and expected
   0x275 raw derived from the strong *stock command* correlation.

The expected 0x275 is diagnostic only; it must not be interpreted as
measured regenerative current, torque, or proof of braking efficiency.

## Findings that remain unresolved

- In an earlier V5.0 recorded event, vehicle aEgo reached +1.293 m/s²
  despite negative PID demand and 0x273 IS_ACCEL=0, near a BRAKING->READY
  transition. This remains unexplained physically.
- Factory-command mapping holdouts on the new November 8 logs had MAE
  ~0.20 km/h, but only recorded speeds up to 78.6 km/h.
- The V4 proposal is based on a counterfactual replay hypothesis, not
  a full plant model; earlier offline V4 showed 251 additional BRAKING
  samples compared with V5.0 READY.
- 0x275 typically tracks factory 0x271 magnitude, but receiving a negative
  value is not independent validation of negative MG2 wheel torque.
- 0x348/0x17A remain uncalibrated, so they are not known HV battery current.
- The branch's Python syntax and unit tests are configured in GitHub Actions
  `.github/workflows/r2v4-closed-track-offline.yml`. A passing run has
  **not** been independently confirmed at the time of this document.

## Offline check before any vehicle use

```sh
cd /data/openpilot
python3 -m py_compile selfdrive/car/dnga/carcontroller.py \
  selfdrive/car/dnga/r2_v4_shadow_controller.py \
  selfdrive/car/dnga/r2_v4_experimental_actuator.py
PYTHONPATH=. python3 -m unittest discover -s selfdrive/car/dnga/tests \
  -p 'test_r2_v4_shadow_controller.py' -v
PYTHONPATH=. python3 -m unittest discover -s selfdrive/car/dnga/tests \
  -p 'test_r2_v4_experimental_actuator.py' -v
```

These are *offline Python checks*. They do NOT validate CAN encodings,
flashed Panda forwarding, ECU acceptance, safety fallback, or the vehicle's
brake and propulsion response.

## Needed evidence

Hardware-in-the-loop verification of the exact flashed Panda and compiled
0.8.13 CAN packer, plus synchronized 0x271/0x273 (bus0 and camera bus2),
0x275 / 0x08C / 0x2C9, true brake/CANCEL transitions, and independently
measured motor current/torque. Calibration-invalid camera yaw should be
resolved separately before enabled lateral control is tested.
