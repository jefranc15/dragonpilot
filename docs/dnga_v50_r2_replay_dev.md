# DNGA V5.0-R2: off-car controller replay and release instrumentation

Base: V5.0 `ce841812a5b17a6bdcac9b4e9deb5235f06978b9`.
**Status: diagnostic development ONLY; not a road-ready longitudinal patch.**

## November 5 native replay findings (39 rlogs)

Previous extraction: 46,586 OP 0x271 20Hz samples and five brief
hydraulic→READY→hydraulic transitions. Corrected native CarControl.actuators
offset is +16 bytes for longitudinal accel; +8 is steering. No R1 stale-handoff
abort matches the five release points.

| Segment | Release s | True downstream accel | Planner accel | aEgo | Reentry gap |
| --- | ---: | ---: | ---: | ---: | ---: |
| 11:32 --3 | 59.225 | +1.250 | -0.265 | -0.684 | 0.606s |
| 11:32 --11 | 32.913 | +0.824 | -0.120 | -0.270 | **0.198s** |
| 14:45 --12 | 11.518 | +1.695 | -0.121 | -0.785 | 0.599s |
| 14:45 --26 (1) | 19.209 | +1.001 | -0.089 | +0.002 | 0.600s |
| 14:45 --26 (2) | 28.459 | +1.038 | -0.187 | -0.639 | 0.601s |

Four gaps near 0.60s are consistent with V5.0's 0.40s reentry
hysteresis + ~0.20s normal entry confirmation; correlation does not prove
the internal reason. The 0.198s gap requires instrumenting the real internal
entry context/exception. No verified emergency exception was seen using only
the external leadOne proxy at reentry.

## Implemented in R2

- `longitudinal_r2_instrumented.py`: OFF-CAR subclass of V5.0.
  Captures true handoff/coast/release counters and lead/guard context
  **before** the base controller clears them, plus hydraulic entry context.
  Calls the inherited methods without changing commands.
- `tests/r2_exact_controller_replay.py`: offline same-input V5.0 vs
  instrumented copy with native 0.8.13 cereal `rlog.bz2` reader;
  compares both controllers' command tuples, then verifies baseline
  brake states against actually recorded source-0 `0x271`.
  Requires at least 95% baseline brake-state agreement.
- `tests/test_longitudinal_r2_audit.py`: regression fixtures for
  all five cases and passive probe import safeguards.

**Critically, carcontroller.py still imports V5.0's
longitudinal_kinematic_guard. R2 is NOT selected for vehicle actuation.**
No Panda, DBC, brake magnitude, ACC mode, SNG, lateral or hybrid feedback
changes have been made. Six pure/offline tests passed in the local
analysis environment (three R2, three prior R1 regressions).

## What was *not* proven

Native 39-segment external-state analysis and pure unit tests were executed,
but the whole original LongitudinalController could not be executed in this
analysis container because its NEOS/0.8.13 cereal runtime and all imports
are not present. The included exact-module runner must still execute on a
compatible **offline** environment and pass its baseline gate. It does
not simulate Toyota ECU/vehicle reaction to changed CAN commands.

Do not interpret this branch as an R2 braking fix. R1 stale-latch
cancellation would not fix these five recorded releases; choosing a safer
policy needs the internal-release replay result first. Avoid making brake
holds or deeper negative ACC_CMD based on unvalidated raw torque fields.

## Offline execution

Inside an isolated worktree/checkout of this branch, with the original
November 5 `rlog.bz2` files extracted (not the CSV shadow replay alone):

```bash
PYTHONPATH=. python3 -m unittest discover \
 -s selfdrive/car/dnga/tests -p 'test_longitudinal_r2_audit.py' -v

PYTHONPATH=. python3 selfdrive/car/dnga/tests/r2_exact_controller_replay.py \
 /path/to/nov5/*/rlog.bz2 \
 --output /tmp/dnga_r2_results
```

Inspect `r2_replay_summary.json` first. If baseline agreement <95%,
do not draw any conclusions about release causes until replay inputs
and startup frame mapping are corrected. Then inspect
`r2_internal_events.jsonl` for the five specific windows, the
0.198-second reentry and unexpected brake release while a lead is closing.

## Required before enabling a new R2 actuation policy

Exact controller replay reproduces recorded 0x271; identify internal
release reason; test normal/urgent/no-lead/SNG/HOLD, driver overrides and
0x275/0x08C physical-brake propulsion veto; then independent brake response
and closed-course validation. Experimental actuation should remain disabled.
