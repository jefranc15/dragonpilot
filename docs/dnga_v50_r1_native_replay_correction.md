# V5.0-R1 November 5 native replay audit — 2026-11-06

Base: `V5.0-R1-long-dev` from V5.0 `ce841812a5b17a6bdcac9b4e9deb5235f06978b9`.

**OFF-ROAD / REVIEW ONLY. Keep `R1_EXPERIMENTAL_LONGITUDINAL_ENABLED = False`.**
No changes to brake output, CAN, Panda, steering, or vehicle control in this audit branch.

## Critical correction to earlier interpretation

The prior R1 handoff analysis incorrectly used the **steer** Float32 from the
Cap'n Proto `CarControl.Actuators` payload as longitudinal `accel`.

From the native V5.0 `car.capnp` and 2026-11-05 rlogs, the serialized
Actuators struct contains these two different Float32 fields:
- payload data offset +8 bytes: `actuators.steer` (normalized lateral command)
- payload data offset +16 bytes: `actuators.accel` (longitudinal demand in m/s^2)

The five historical "negative PID demand" values correspond to `steer`.
At the exact release samples, correctly decoded `actuators.accel` is
**positive in all five cases**, while `longitudinalPlan.accels[0]` stays
negative. These are genuine planner/downstream disagreement events and explain
why the normal positive-PID hydraulic-to-coast rule may be taking effect.

| 0x271 last BRAKING mono time (s) | steer, previously mislabeled PID | true actuators.accel (m/s²) | planner.accels[0] (m/s²) | delay until BRAKING resumes |
| ---: | ---: | ---: | ---: | ---: |
| 847.653 | -1.000 | +1.695 | -0.121 | 0.599 s |
| 1695.333 | -0.431 | +1.001 | -0.089 | 0.600 s |
| 1704.583 | -1.000 | +1.038 | -0.187 | 0.601 s |
| 1790.305 | -0.196 | +1.250 | -0.265 | 0.606 s |
| 2243.985 | +1.000 | +0.824 | -0.120 | 0.198 s |

These are independent raw Cap'n Proto actuator values, reverified by
re-reading the five serialized `CarControl` samples, rather than copied from
a prior summary.

## What was actually completed offline

- Decompressed all 39 November 5 rlog segments from `realdata(2).zip`.
- Reconstructed 46,586 20 Hz OP-transmitted 0x271 brake-command samples.
- Aligned `carState` vehicle speed/acceleration, `carControl.actuators.accel`,
  `longitudinalPlan`, `radarState`, and OP 0x271/0x273 command bits.
- Confirmed 35 braking episodes and the five previously described brief
  brake/off/brake transitions. All five had planner hasLead=true through gap.
- At the five release points the existing R1 `should_abort_handoff`
  predicate evaluates **False** because `pid_accel <= -0.05` is not true.
  This excludes the R1 stale-handoff abort as a fix for these five releases;
  internal latch values were not recorded, so it cannot identify the definitive
  release cause.
- An external-input **shadow** of the R1 early coast predicate found
  18 OP READY/IS_ACCEL samples across four segments that would be candidates
  for neutral framing (under the visible-field proxy).
  The first qualifying frame during the marked weak-braking approach is at
  ~475.90 s, about 0.65 s before the 0x271 hydraulic onset ~476.55 s.
- All observed moving BRAKING samples have IS_ACCEL=0. Eleven HOLD/CREEPING
  overlap samples are intentional state encoding, not evidence of acceleration.

**This is an external-input shadow analysis, NOT a full executable V5.0/R1
controller same-input replay**. Hidden internal states `handoff_active`,
`release_counter`, `brake_reentry_frame` are not recorded in the rlogs.
The shadow uses radar status as an approximation for internal trusted-lead
state and cannot predict closed-loop Toyota braking, brake pressure,
stopping distance, or R1's actual vehicle effect.

## Decision

Do **not** enable the R1 flag and do **not** merge the R1 abort candidate as
a confirmed chatter fix. The evidence is consistent with the *intended*
coast-first handoff operating while the planner stays mildly negative and
actual vehicle deceleration is stronger than the desired slowing; later
planner demand intensifies and hydraulic braking re-enters. We must distinguish
that from a delayed/hard release with an instrumented controller replay.

Next safe engineering task:
1. Add replay-only instrumentation to log actual release reasons and internal
   handoff latch, confirmation counters, and brake re-entry deadlines.
2. Execute the complete controller on recorded 100Hz CarState + 20Hz planner
   and radar messages on an isolated 0.8.13 environment, comparing V5.0
   versus any revised candidate. Preserve the 0x275/0x08C propulsion veto.
3. Only consider a new chatter policy after it demonstrates no unsafe lead
   stopping/urgent response regression; no public-road activation until
   physical brake response is independently validated.

## Files

Pure test: `selfdrive/car/dnga/tests/test_r1_wire_release_regression.py` in
this branch. Exact 20Hz CSVs, extractor, and full source evidence are in the
downloadable R1 replay package supplied in the conversation.
