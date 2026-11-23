# DNGA V7.1-R1 — inactive camera ACC decoding + historical release diagnostics

**Status: SOURCE/SHADOW-ONLY. NOT A VALIDATED ROAD-DRIVING RELEASE.**

Base: `V7.1-OEM-follow` (commit `19753f09f40f8de6139a4ed51eebd423aefa1b8e`).
New branch: `V7.1-R1`.

## Changes

1. `stock_acc_decode.py`: standalone, unit-testable OEM camera 0x273
   decoder. All checksum-valid **inactive** frames (camera engagement bit
   is clear) return immediately as `(False,False,False,False,0.0)`,
   regardless of constants seen only on engaged frames. Engaged frames
   still require both previously observed constant bits. Bad checksums,
   lengths and unknown active signatures are rejected. This prevents
   an ordinary inactive frame being misclassified as corrupt and leaving
   OP cruise engagement cached for the 150 ms freshness interval.
2. `interface.py`: uses the shared decoder. Any malformed/invalid
   received camera 0x273 immediately invalidates the cached camera
   cruise request. This is a fail-closed display/lateral-engagement
   indication, *not an authenticated ECU actuation acknowledgment*.
3. `oem_follow_shadow.py`: optional `historical_op_brake_state`
   supports **offline V6** 0x271 BRAKING→READY release replay, even when
   Toyota's original camera remained READY throughout. A positive
   aEgo excursion during the release-monitor window can now be
   categorized as `POST_BRAKE_SURGE_WARNING` in the *historical* replay.
   The argument is not used or supplied by the live V7/V7.1 controller,
   which has no OP longitudinal transmit path.
4. `carcontroller.py`: logs diagnostic turn-plan source, freshness age,
   received sample count, and subscription error count in
   `V71_OEM_FOLLOW_SHADOW` lines. The source is already subscribed at
   the 20 Hz shadow evaluation cadence; these fields make missed
   messages and stale/missing OP vision turn input distinguishable.

## Original hard-launch evidence and limits

The November 17 V6 segment 27 investigation was documented in
`docs/dnga_v60_highway_handoff_20261118.md`:
OP bus-0 0x271 released from BRAKING to READY around route
t=1664.5916, while Toyota's original camera bus-2 0x271 remained
READY and 0x273 reported `IS_ACCEL=1`. Peak measured acceleration
was +1.89 m/s² without driver throttle; the downstream OP PID
was negative while vehicle acceleration was positive.

R1's regression fixture encodes that **documented pattern** rather
than the previously inaccurate camera-BRAKING→READY transition.
The original binary rlogs/decoded time-aligned rows are not embedded
here, so successful CI is **NOT a replay of the full 7 recorded rlogs**
and cannot verify timing or causality beyond the fixture.

## Validation and nonclaims

`test_v71_r1.py` checks:
- Four inactive signature combinations, including missing engaged
  constant bits while the OEM enabled bit is clear
- Inactive frame with stale mode/lead/target bits
- Enabled-frame positive contract and enabled-frame rejections
- Checksum and message length guards
- Immediate active→inactive decoding transition
- Fail-closed raw interface source contract
- Summary-derived Nov 17 historical OP release even though factory
  camera is READY
- No false release from an unchanged READY state
- Original factory-braking release monitor unchanged
- No historical OP command wired into the live controller
- Unchanged factory-forwarding and steering-only safety contracts

OP turn-plan reception diagnostics need on-device **read-only log**
verification to prove runtime messages are actually arriving. Static
CI cannot prove a vision model is running or producing turn demands.

Crucially, this release does NOT:
- Send OP braking 0x271 or ACC target 0x273
- Alter factory camera forwarding 0x271/0x273/0x274
- Change `openpilotLongitudinalControl=False`
- Change Panda firmware, board safety model `dngaStock=28`,
  camera braking magnitude, pump, regen, hybrid ECU, or curve actuation
- Fix the unvalidated V7 Panda steering torque/rate and driver-override
  safety checks

No road testing is recommended until those Panda and vehicle-specific
safety weaknesses have been independently corrected and bench tested.

## Parked device: source checkout and tests only

```sh
cd /data/openpilot || exit 1
git status --short
test -z "$(git status --porcelain)" || { echo "Please resolve uncommitted changes"; exit 1; }
git fetch origin V7.1-R1:refs/remotes/origin/V7.1-R1 || exit 1
git switch V7.1-R1 2>/dev/null \
  || git switch -c V7.1-R1 --track origin/V7.1-R1 || exit 1
git merge --ff-only origin/V7.1-R1 || exit 1
git rev-parse --short HEAD
python3 selfdrive/car/dnga/tests/test_v71_r1.py -v
```

The checkout does not flash Panda firmware or establish road safety.
Do not restart on-road testing solely because the offline checks pass.

## Remaining work

- Inspect actual V7.1-R1 rlog or saved diagnostic messages for
  `V71_OEM_FOLLOW_SHADOW ... plan_source=... plan_age=... plan_count=...`.
- Run the new decoder against the original November 8 *raw* camera
  frames to confirm the historical 2,729 inactive frames, not just the
  unit-tested inactive signature combinations.
- Replay the **full** November 17 aligned `0x271` bus0/bus2
  frames, Panda state, PID demand, aEgo and RPM; the summary-derived
  fixture is useful but insufficient.
- Fix Panda steering torque/rate limits, checked engagement, driver
  override and protective RX validation before considering road use.
- Only then assess hypothetical braking smoothing and turn arbitration
  in bench/HIL using independent ECU-side instrumentation.
