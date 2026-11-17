# Corrected November 5 same-input replay results (2026-11-06)

**OFF-ROAD ANALYSIS ONLY.** Keep `R1_EXPERIMENTAL_LONGITUDINAL_ENABLED = False`.
No longitudinal actuation, lateral, CAN, or Panda changes.

## Source and method

The original 39 November 5 `rlog.bz2` segments were decoded to 46,586
20Hz source-0 OP `0x271` transmissions. Alignments include native
`carState`, `carControl.actuators.accel` (not steer), `longitudinalPlan`,
`radarState`, and matching source-0 `0x273`.

The **exact pure R1 `should_abort_handoff()` predicate** was executed against
the five recorded releases. Other V5.0/R1 preconditions were screened using
external state and a simulated 20Hz debounce, with radar+planner lead visibility
as proxies. A complete execution of the original controller's unlogged internal
state and a physical vehicle counterfactual **was not performed**.

## Five short re-entries

| Segment | Release offset | True longitudinal accel | Planner accel0 | Time to re-entry | R1 abort at / preceding .4 s | V5.0 coast 4-cycle visible proxy |
|---|---:|---:|---:|---:|---|---|
| 11:32 `--3` | 59.225 s | +1.250 | -0.265 | 0.606 s | false / false | true |
| 11:32 `--11` | 32.913 s | +0.824 | -0.120 | 0.198 s | false / false | true |
| 14:45 `--12` | 11.518 s | +1.695 | -0.121 | 0.599 s | false / false | false |
| 14:45 `--26` | 19.209 s | +1.001 | -0.089 | 0.600 s | false / false | true |
| 14:45 `--26` | 28.459 s | +1.038 | -0.187 | 0.601 s | false / false | true |

The earlier negative PID interpretation was the wrong Cap'n Proto field:
it was `actuators.steer`. Real `actuators.accel` was positive in all cases.

## Extended screen

- 2,305 ordinary moving BRAKING samples and 86 CREEPING/HOLD samples.
- R1 stale-handoff abort external predicate could hold on **426** other
  moving BRAKING samples. Internal `handoff_active` was not logged, so the
  number of actual altered releases is not measurable from rlogs alone.
- R1 early-coast three-cycle screen finds **21** READY/IS_ACCEL samples across
  five small windows. The actual R1 policy also requires internal lead trust,
  freshness, and no urgent/curve/hold; this is *potential* eligibility only.
- Two windows span `14:45 --5` and `--6`, before the weak-braking incident.
  Neutral `0x273` framing does not itself command earlier or stronger
  hydraulics, so no stopping-distance benefit can be inferred.
- The **0.198 s** re-entry in `11:32 --11` is shorter than the nominal
  0.4 s normal re-entry guard. It requires internal reason/exception tracing.

## Decision

Do not enable R1 as a fix for the five release/re-entry events. The observed
positive downstream PID demand is consistent with the existing intentional
coast-first handoff. Internal release-latch state was not recorded, so this
is **not** proof of a stale latch.

To establish a safe R2, instrument `handoff_active`, `release_counter`,
`coast_release_counter`, `brake_reentry_frame`, chosen release/entry cause,
lead freshness/trust and urgent/SNG exceptions in an isolated exact-controller
replay. Preserve physical-brake veto and do not change braking actuation based
on proxy replay alone. No public-road deployment of unvalidated longitudinal
changes.

The full offline input timeline, executable predicate replay, CSVs and test
fixtures are included in the attached conversation audit ZIP.
