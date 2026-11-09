# DNGA Toyota Yaris Cross HEV — v6.0 refactor

**Branch:** `v6.0` — experimental, not validated for active driving.

The car port has one command source per cycle. `CarController` chooses either
the existing V5.0 longitudinal implementation or the integrated R2 controller;
it does **not** compute V5.0 and then overlay an R2 shadow/actuator command.
The normal moving-brake R2 policy begins at `0x271 MAGNITUDE=1216`, pump
`-0.4`, then shapes demand progressively and sends a below-vEgo `0x273`
speed target. The factory-inspired `READY / 1224 / -0.4` release remains
experimental and not vehicle-response validated.

## Runtime file structure

| File | Responsibility |
|---|---|
| `carcontroller.py` | Steering limits and CAN scheduling; one longitudinal command producer; final brake/propulsion veto; HUD |
| `longitudinal.py` | Integrated R2 proposal, moving-brake state, 1216 entry, `0x273` coordination, release and brake feedback eligibility |
| `longitudinal_v50_legacy.py` | Historical V5.0 base, preserved for low-speed compatibility and rollback |
| `longitudinal_kinematic_guard.py` / `longitudinal_op_only.py` / `longitudinal_atomic_release.py` | Historical V5.0 compatibility chain; no extra on-road shadow producer |
| `dnga_hybrid_feedback.py` | Validated frame checksums, 0x275/0x08C freshness and telemetry; torque-related values are diagnostic-only |
| `carstate.py` | CAN state, ACC MAIN/SET/RES, brake, pedal and LKAS |
| `interface.py` | CarParams, fingerprint, state and passive stock-camera CAN observation |
| `dngacan.py` | Outgoing 0x1D0/0x271/0x273/0x274 packing |
| `values.py` | Fingerprints, longitudinal/steering values and brake states |
| `radar_interface.py` | Standard inherited no-radar interface |

DBC: `opendbc/dnga_hev.dbc`. No Panda firmware/safety changes here.

The previous `r2_v4_shadow_controller.py`,
`r2_v4_experimental_actuator.py`, and `r2_v4_direct_authority.py`
modules and their obsolete tests have been removed. Their Git history
remains accessible on the V5.2 branches.

## Active-command contract

`carcontroller.py` has `V6_R2_ACTIVE_CAN = False` **by default**:
- `False`: the original V5.0 controller alone generates the outgoing
  longitudinal command. It still uses the original neutral-vEgo target
  during hydraulic braking. No R2 shadow runs.
- `True` (UNVALIDATED, DO NOT USE IN TRAFFIC): the integrated
  `longitudinal.LongitudinalController` becomes the only moving-brake
  command producer, retains the below-vEgo R2 target under the final
  interlock, and hands stop-and-go/low-speed responsibility to the old
  compatibility controller. The old V5.0 moving-brake authority is not
  used as an R2 brake floor.

**Do not change the flag for driving** until HIL, native 0.8.13 CAN encoding,
brake/CANCEL tests, low-speed takeover, independent pedal/ECU response
measurements, and safety review are complete.

## What the logs support — and do not support

The genuine stock ACC samples demonstrated an initial mild factory braking
magnitude `1216` with a 0x275 raw request association around `-163`; the
correlation is with the factory brake command, not independently verified
MG2 wheel torque or actual regenerative current. The below-vEgo 0x273
target was present in stock samples before the first hydraulic command, but
the current R2 fit is limited by the stock sample's speed domain.

The November 6 V5.0 no-lead curve log captured **+1.293 m/s²** measured
acceleration after a BRAKING→READY transition while OP transmitted neither
positive ACCEL mode nor driver gas. The physical source is unresolved. No
algorithmic refactor can certify this interaction without physical testing.
The existing Panda safety hook was previously permissive and must be
independently assessed.

## Validation

GitHub Actions `.github/workflows/v60-r2-offline.yml` syntax-checks the
car port and runs isolated R2 logic tests without compiled cereal/DBC.
Passing these checks does not simulate the ECU, validate actual CAN bytes,
prove emergency stopping behavior, or qualify the branch for road use.

Review command (parked, no checkout):

```sh
cd /data/openpilot
git fetch origin refs/heads/v6.0:refs/remotes/origin/v6.0
git diff --stat V5.2-R2-1216-direct-dev..origin/v6.0
git show origin/v6.0:selfdrive/car/dnga/README.md
```

The active device checkout, ECU firmware, Panda safety and V5.0 branch are
unchanged by the commits on `v6.0`.
