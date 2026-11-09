# DNGA Toyota Yaris Cross HEV — v6.0 (R2 on V5.0 scaling)

**Development branch, not road-ready:** R2 live CAN selection is disabled.
The earlier v6 R2 rewrite used a separate low-speed controller and did not
preserve the full V5.0 brake scaling. Both issues were corrected.

## Normal OpenPilot car port layout

- `carcontroller.py`: steering limits, CAN scheduler, HUD, 0x271/0x273
  encoding call sites and final brake/propulsion veto
- `longitudinal.py`: **one** V5.0-derived longitudinal state machine.
  R2 replaces moving-brake/coast-first arbitration, starting at magnitude
  1216, then uses inherited V5.0 _apply_brake_target filter/ramp, existing
  speed-dependent caps and measured-deceleration governor.
  The V5.0 base continues ALL low-speed stop-and-go, HOLD, CREEPING,
  engagement/lead/feedback and propulsion-state updates in the same object
- `longitudinal_v50_legacy.py`, `longitudinal_kinematic_guard.py`,
  `longitudinal_op_only.py`, `longitudinal_atomic_release.py`: the
  retained original V5.0 state machine/compatibility methods and fallback.
  They are parents of the R2 controller, **not separate running controllers**
- `dnga_hybrid_feedback.py`: read-only 0x275/0x08C feedback and freshness
- `carstate.py`, `interface.py`, `values.py`, `dngacan.py`,
  `radar_interface.py`: standard port roles; DBC `opendbc/dnga_hev.dbc`

Previous standalone R2 shadow, actuator and replay modules were removed.
Their history is available from V5.2 branches.

## Braking algorithm

1. The inherited V5.0 code updates the SAME planner/radar/lead/session
   state once at the normal 20 Hz longitudinal CAN cadence.
2. **Below 5.6 m/s (~20 km/h), standstill, stop-hold, armed SNG or SNG
   release:** the original V5.0 `_update_hydraulic` path runs unmodified.
   There is no second low-speed controller or late state migration.
3. Above that domain, the R2 moving-brake entry accepts legitimate
   planner deceleration from a lead, selected vision turn (while above
   the vision-turn speed), or user lower cruise set speed. It does **not**
   require a V5.0 lead→coast→hydraulic handoff.
4. On the first ordinary moving BRAKING request, `apply_brake = 0.08`
   encodes with the ORIGINAL V5.0 `encode_hev_brake` to
   `BRAKE_STATE=0x21, MAGNITUDE=1216, PUMP=-0.4`.
5. The NEXT updates call the original V5.0 `_apply_brake_target`
   filter (0.08) and speed/lead/curve caps, then use a *normal-only*
   comfort rise of up to +0.015 per 20 Hz update. This is 75% of the
   median +0.020 per 50 ms increase observed in factory braking.
   V5.0 urgent (+0.015), emergency (+0.030), and physical
   deceleration-governor paths remain unattenuated. Release continues
   with V5.0's -0.015 step; the first magnitude is not fixed.
6. During R2 moving BRAKING, the lower 0x273 `ACC_CMD` is based on
   the newly fitted stock gap (-0.3159 + 7.0102 * strength +
   0.03394 * speed_kph), reduced to **85%** for normal comfort
   braking, not for urgency. Target slewing remains +0.18/-0.35
   km/h per 20 Hz update. No direct motor torque/regenerative
   current control is asserted or proved. See
   `docs/dnga_v60_stock_acc_comfort_fit.md` for data and limitations.
7. Normal moving release ramps brake strength down with the V5.0
   release rate/confirmation, then proposes a 24-update
   READY/1224/-0.4 phase. This release hypothesis remains UNVALIDATED
   against actual hybrid ECU behavior and previous engine-rev events.
8. V5.0's emergency lead supervision, low-speed stop, normal accelerator
   gating and HEV feedback rules remain inherited. Current R2
   start/scale behavior nevertheless needs independent validation for
   critical stopping and unexpected feedback.

## Safety status

`carcontroller.py`: `V6_R2_ACTIVE_CAN = False`.

- With FALSE (current repo default), CAN output uses the original V5.0
  longitudinal controller and neutral-vEgo target during its braking.
  R2 is not executed/shadowed in this mode.
- With TRUE, the R2 controller would replace normal moving-brake
  arbitration and change 0x271/0x273 requests. **Do not enable for an
  engaged public-road drive without verifying a physical safe operating
  envelope and ECU feedback at every speed.**

Passing the isolated tests only demonstrates source-level behavior. It
does not verify actual DBC packing on the OnePlus 3T, Panda safety,
OEM ECU response, regenerative torque, emergency deceleration,
release/re-entry, or a safe SNG boundary handoff. Earlier November 6
logs recorded unexplained acceleration after braking (+1.293 m/s²),
which has not been physically resolved. Missing or inconsistent 0x275
response still lacks a validated brake-failure/degraded-mode strategy.

## Offline verification

`.github/workflows/v60-r2-offline.yml` syntax-checks the DNGA port,
and tests 1216 entry, V5.0 rate limits, highway/curve envelope,
source authority, release ramp and inherited low-speed call flow.

Review only, without changing the installed phone branch:

```sh
cd /data/openpilot
git fetch origin refs/heads/v6.0:refs/remotes/origin/v6.0
git diff --stat origin/v6.0-pre-v50-scaling..origin/v6.0
git show origin/v6.0:selfdrive/car/dnga/longitudinal.py
```
