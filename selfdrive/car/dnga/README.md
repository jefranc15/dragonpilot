# DNGA Toyota Yaris Cross HEV — v6.0 (R2 on V5.0 scaling)

**Experimental integration, not road-ready:** one DNGA longitudinal controller
is selected unconditionally in `carcontroller.py`. The moving-brake behavior
is experimental and sends live CAN when the vehicle is engaged. The historical
low-speed stop-and-go/HOLD/CREEPING logic remains inherited, running on the
SAME instance with its existing state; there is no fallback controller
selector or second low-speed controller.

## Normal OpenPilot car port layout

- `carcontroller.py`: steering limits, CAN scheduler, HUD, 0x271/0x273
  encoding call sites and final brake/propulsion veto
- `longitudinal.py`: **one** inherited longitudinal state machine.
  The moving-brake behavior starts at magnitude 1216, then uses the
  historical _apply_brake_target filter/ramp, speed-dependent caps
  and measured-deceleration governor. The inherited low-speed
  stop-and-go, HOLD, CREEPING, engagement/lead/feedback and
  propulsion state continues in that same object
- `longitudinal_v50_legacy.py`, `longitudinal_kinematic_guard.py`,
  `longitudinal_op_only.py`, `longitudinal_atomic_release.py`: the
  retained historical braking/SNG methods in the inheritance chain.
  These are **not separate running controllers or a runtime fallback**
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
4. On the first **ordinary** moving BRAKING request, `apply_brake = 0.08`
   still encodes with the original V5.0 encoder to
   `BRAKE_STATE=0x21, MAGNITUDE=1216, PUMP=-0.4`.
   A trusted, already-authorized lead with strong planner braking may instead
   enter at up to 0.12; geometry-confirmed emergency lead braking at up to
   0.16. Both respect the actual requested demand and inherited speed/lead
   caps. These urgent initial requests remain physically unvalidated.
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
   release rate/confirmation, then proposes a 16-update (80 frames,
   0.8-second) READY/1224/-0.4 phase. This shortened release hypothesis
   remains UNVALIDATED against actual hybrid ECU behavior and previous
   engine-rev events. Historical SNG release remains unchanged.
8. V5.0's emergency lead supervision, low-speed stop, normal accelerator
   gating and HEV feedback rules remain inherited. Current R2
   start/scale behavior nevertheless needs independent validation for
   critical stopping and unexpected feedback.

## Safety status

`carcontroller.py` constructs only `LongitudinalController()`, imported
from `longitudinal.py`, without a V5/R2 selector.

- During authorized **moving braking**, the controller uses experimental
  progressive 0x271 requests and a lower 0x273 ACC target. The final
  brake/propulsion interlock retains that lower speed target only in this
  moving phase, and vetoes positive propulsion while braking.
- During **low-speed/SNG, HOLD and CREEPING**, it uses historical inherited
  methods and the old neutral 0x273 hydraulic target. This preserves SNG
  CAN behavior instead of applying a moving-only interlock to every speed.
- During **post-release handoff**, it neutralizes both 0x273 mode bits while
  physical brake feedback or the inherited handoff state blocks acceleration.

**Do not treat this integration as road-ready.** It has not been verified
against actual ECU response, safe stopping distance, or the earlier RPM-surge
failure. R2's 16-step READY/-0.4 moving release and first-frame urgent
0.12/emergency 0.16 maximum commands remain unvalidated. The Panda hook also lacks independent
command-content and disengagement enforcement.

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
source authority, normal/urgent/emergency initial request envelopes, the
16-step release pump interval and inherited low-speed call flow.

Review only, without changing the installed phone branch:

```sh
cd /data/openpilot
git fetch origin refs/heads/v6.0:refs/remotes/origin/v6.0
git diff --stat origin/v6.0-pre-v50-scaling..origin/v6.0
git show origin/v6.0:selfdrive/car/dnga/longitudinal.py
```
