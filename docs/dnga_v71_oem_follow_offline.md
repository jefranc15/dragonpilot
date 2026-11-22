# V7.1-OEM-follow: Toyota camera longitudinal + OP vision-turn **shadow**

**Status: OFFLINE/OBSERVATIONAL DEVELOPMENT. DO NOT ROAD-TEST experimental control.**

Base: V7.0-stocklong, with NO changes to `panda/board/safety/safety_dnga_stock.h`,
`cereal/car.capnp`, or OEM CAN forwarding. Panda safety mode remains
`dngaStock=28`. OP `openpilotLongitudinalControl=False`.

## Why

November 17 V6.0 rlogs recorded sustained highway revving and a no-pedal
post-braking +1.89 m/s² launch. November 8 stock ACC logs contained 18
identified brake episodes (the sampled initial OEM brake median about
0.275 decoded strength), while V7's example shadow started at 0.025:
transmitting that suggestion would risk substantial underbraking.
The November 12 turn sample recorded OP `source=turn` with camera
`0x271 READY`, no OEM lead, and OEM `0x273 IS_ACCEL` active.
Curve braking is NOT safe to add by changing 0x271 alone.

## What was implemented

1. `oem_follow_shadow.py` is a pure, read-only, deterministic decision
   evaluator. It accepts decoded camera 0x271 and 0x273, OP
   `longitudinalPlan` turn source/turn speed/near-term acceleration,
   vehicle speed and measured acceleration, and driver pedal state.
2. `carcontroller.py` subscribes to `longitudinalPlan` at the existing
   20Hz ACC evaluation cadence and computes an OEM-follow decision:
   `OEM_LEAD`, `OEM_DECEL`, `CURVE_PROPULSION_CONFLICT`,
   `CURVE_UNAUTHORIZED`, `CURVE_SHADOW_CANDIDATE`,
   `RELEASE_MONITOR`, `POST_BRAKE_SURGE_WARNING`, etc.
   Factory camera lead always wins over an OP vision-turn suggestion;
   OP `hasLead` is never injected into `IS_LEAD`.
3. The original `StockBrakeShadow` remains and V7.1 quantifies its
   hypothetical magnitude weakening and hypothetical proportional
   pump response. These are **not physical brake/regen predictions**.
   Significant proposed weakening (>20% of camera magnitude demand)
   is flagged; no software concludes it is safe.
4. A contiguous 0x271/0x273 camera pair is required (both received
   within 15 control frames, <=5 frames apart). Driver override,
   camera loss, stale plan, mismatching lead sources and post-brake
   recovery preclude new turn proposals. The first 0.8 seconds after
   brake release are watched for an uncommanded positive acceleration
   response; no braking or release action is taken.
5. At most once per second plus mode changes, shadow diagnostics are
   emitted as `V71_OEM_FOLLOW_SHADOW ... (NOT TRANSMITTED)`.

The *only* transmitted command remains OP 0x1D0 steering:
`carcontroller.py` contains no 0x271, 0x273, or 0x274 transmit path.
Panda forwards OEM camera 0x271/0x273/0x274 unchanged, as in V7.0.
This is NOT an active OEM-command-replay or curve-braking controller.

## Intended prioritization

- Invalid/stale camera pair or driver pedals: no turn proposal
- OEM ACC disabled: no turn proposal
- Factory OEM lead present: OEM lead authority
- OEM 0x271 braking or independent OEM decel: OEM authority
- Brake-release window: observation-only recovery/surge warning
- OP lead without OEM lead: lead source disagreement; no spoofing
- Vision turn while OEM `IS_ACCEL=1`: conflict; **never add a brake**
- Vision turn while OEM has no decel: unauthorized standalone braking
- Vision turn while OEM already `IS_DECEL=1`, no active 0x271:
  record *hypothetical* lower turn speed only after three consistent
  20Hz plan samples. Still no command is transmitted.

A camera `IS_LEAD` bit is an OEM-reported state, NOT an independently
verified target distance/relative speed/TTC and NOT proof that the ECU
accepted the camera command. There are known 0x271/0x273 transitions
where the individual messages do not update atomically. The shadow
uses freshness and skew checks but cannot prove synchronized ECU action.

## CI and offline tests

```sh
python3 -m py_compile selfdrive/car/dnga/carcontroller.py selfdrive/car/dnga/oem_follow_shadow.py
python3 selfdrive/car/dnga/tests/test_v71_oem_follow_shadow.py -v
python3 selfdrive/car/dnga/tests/test_v70_stocklong_shadow.py -v
python3 selfdrive/car/dnga/tests/test_v60_highway_handoff_audit.py -v
```

Test scenarios explicitly model previously observed **patterns** (Nov 12
turn conflict; Nov 17 post-release surge), rather than claiming to
replay the original binary rlogs. Existing analysis is separately
preserved in historical evidence files.

## Parked-device *source checkout only*

Only with the vehicle fully parked and device not driving:

```sh
cd /data/openpilot || exit 1
git status --short
test -z "$(git status --porcelain)" || { echo "Uncommitted changes: stop"; exit 1; }
git fetch origin V7.1-OEM-follow || exit 1
git switch V7.1-OEM-follow 2>/dev/null \
  || git switch -c V7.1-OEM-follow --track origin/V7.1-OEM-follow || exit 1
git merge --ff-only origin/V7.1-OEM-follow || exit 1
git rev-parse --short HEAD
python3 selfdrive/car/dnga/tests/test_v71_oem_follow_shadow.py -v
```

**Stop after these source-only checks.** This does not validate the
existing V7 Panda mode or authorize active driving. No new Panda firmware
flash is necessary to run Python tests, because the safety firmware code
has not changed from V7.0.

## Requirements before any real command rewriting

- Vehicle-specific Panda steering torque/rate and driver override
  enforcement (current V7 stock Panda hook is not road qualified).
- Independent ECU-side CAN capture verifying factory camera forwarding,
  command counters, checksums, AEB and disengagement; sensor/firmware
  freshness and verified engagement.
- Correct HEV 0x271/0x273 state, mode, timing and pump relationships.
  Preserve brake response in closing-lead and emergency events.
- Bench/HIL validation of decreased brake magnitude with matched
  deceleration/stopping distance, engine RPM, SOC and motor/engine torque.
- Bench/HIL validation of OP turn-request arbitration while OEM
  `IS_ACCEL` is active, plus anti-surge recovery after turn/brake release.

Do not feed `hypothetical_brake_strength`, `hypothetical_pump`,
or `hypothetical_curve_target_kph` into actuators or CAN. Earlier
V6 hard launches are unresolved and a successful CI run is not
a brake or steering safety certification.
