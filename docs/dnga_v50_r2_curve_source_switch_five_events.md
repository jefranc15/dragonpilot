# R2 evidence: no-lead curve planner-source flicker and hydraulic release

**Off-road diagnostic finding; NOT a new braking policy.**
Source: V5.0 2026-11-06 route `2026-11-06--02-08-06`,
segments 85–87, native rlogs decoded to 20 Hz OP source-0 messages
and synchronized vehicle/planner states.

## Five repeated observed sequences

When braking is active with `plan_hasLead=0` and radar leadOne
absent, planner `longitudinalPlanSource` changes from `turn` (5)
to `cruise` (0) shortly before OP hydraulic BRAKING (0x21) changes
to READY (0x01).

| Segment | Turn→cruise (s) | Hydraulic READY (s) | Delay (s) | Planner accel at switch | Downstream accel at switch |
|---|---:|---:|---:|---:|---:|
| --86 (a) | 37.527 | 37.994 | .467 | -.155 | +.027 |
| --86 (b) | 46.033 | 46.501 | .468 | -.184 | -.573 |
| --86 (c) | 58.884 | 59.435 | .551 | -.318 | -.523 |
| --87 (a) | 34.405 | 34.854 | .449 | -.256 | -.437 |
| **--87 (b)** | **43.763** | **44.274** | **.511** | **-.311** | **-.469** |

The ordinary `P.RELEASE_CONFIRM_COUNT=10` at 20 Hz corresponds
to ~0.50 seconds. The actual V5.0 curve-braking branch in
`longitudinal.py` has:

```python
if self.curve_brake_active:
  ...
  low_demand = (
    (not plan.curve_active)
    or plan.brake < P.CURVE_BRAKE_RELEASE
    or curve_speed_error <= P.CURVE_SPEED_ERROR_RELEASE
  )
...
elif self.handoff_active or low_demand:
  self.release_counter += 1
```

In `_get_plan`, `plan.curve_active` requires source `turn`.
The `turn`→`cruise` switch therefore makes `low_demand` true
**if `self.curve_brake_active` is still true at that instant**,
without testing current negative downstream PID. The five delays are
consistent with normal 10-frame release confirmation. This is a
**strong mechanistic hypothesis**, but the actual `curve_brake_active`
and `release_counter` before each release were not logged in the
production drive. A source switch alone cannot prove this was the
branch taken. The new timestamped R2 internal probe can verify it.

### The severe --87 case

- Time at source switch: 43.763 s; 0x271 BRAKING, lead absent;
  planner request -0.311 m/s², downstream -0.469 m/s²,
  RPM 2,716, aEgo -0.298 m/s².
- Time at release: 44.274 s; READY, source `turn` has returned,
  RPM 2,711, ~75.85 km/h.
- Next 1.5 s: RPM reaches ~3,046 (nearest OP frame, raw CAN
  sample peak 3,047), measured acceleration +1.293 m/s²,
  and vehicle speed rises ~5.86 km/h.
- OP's 0x273 is neutral during the post-release acceleration
  and driver accelerator CAN 0x277 remains zero.
- Factory-camera bus-2 0x273 requests IS_ACCEL, but V5.0's
  **repository** Panda safety hook blocks forwarding that ID to
  bus 0. The actually flashed Panda firmware must be checked
  separately; the camera request is not proof of competing
  ECU torque command.

## Additional replay tooling

- R2 recorded controller probe now timestamps staged-release start
  and hydraulic clears and captures `curve_brake_active`,
  `source`, speed error, RPM and pedal state.
- `r2_exact_controller_replay.py` exports
  `r2_20hz_state_trace.csv` and `r2_panda_states.jsonl`
  in addition to its original event/command evidence.
- `tests/r2_run_dual_offline.sh` executes the corrected Nov 5
  and Nov 6 native rlog ZIP replays sequentially in an isolated
  controller worktree, and creates one compressed evidence bundle.
- The November 5 annotated segment ordering was fixed. This
  is important because the initial 99.69% baseline had run
  annotated --14/--15 *before* --0 and cannot verify internal
  sequential state history for that route until rerun.

## Status and safety

The external-input evidence review and all five source-switch
screens are complete. **The full native controller replay of
the corrected Nov 5 order and new Nov 6 archive must still run
on a compatible 0.8.13 Python runtime.** A passing 95% brake-state
agreement gate is necessary but not sufficient for a physical
vehicle-response conclusion.

Do not modify production `longitudinal.py`, steering, Panda,
regen offset, friction magnitude, release timers, or safety veto
based on this correlation alone. Any attempted source-flicker
hold logic must have defined safety exits (lead loss, driver
brake, disengagement, urgent lead, standstill, stable noncurve
plan, and fresh-trust expiration), extensive offline regression,
and independent closed-course validation before road use.
