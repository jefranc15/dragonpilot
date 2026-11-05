# DNGA V5.0-R1 experimental longitudinal candidate — OFFROAD ONLY

Base: `V5.0`, commit `ce841812a5b17a6bdcac9b4e9deb5235f06978b9`.

**NOT APPROVED FOR ENGAGED VEHICLE USE.** The candidate is present in code but intentionally gated by
`R1_EXPERIMENTAL_LONGITUDINAL_ENABLED = False` in `selfdrive/car/dnga/longitudinal_kinematic_guard.py`.
Do not change this flag for on-road driving. No V5.0 lateral, DBC, Panda, TX guard,
friction/0x275 veto, brake magnitude, urgent-entry, or CAN forwarding changes.

## Evidence correction: planner lead does NOT flicker in chatter gaps

Native Cap'n Proto rlogs were re-decoded for `longitudinalPlan.hasLead` (data word 0,
bit 2), and `longitudinalPlanSource` (word 0 high 16 bits). **All five**
November 5 20-Hz release/reentry gaps preserve `hasLead=True` at each available
20-Hz plan sample. This refutes our earlier proposed planner-lead dropout bridge.
No artificial lead trust or missing-lead grace has been added to R1.

| Route time of last 0x271 BRAKING sample | PID accel | Planner accel[0] | Gap until BRAKING | lead true |
| --- | ---: | ---: | ---: | --- |
| 847.653s | -1.000 | -0.121 | .599s | yes |
| 1695.333s | -.431 | -.089 | .600s | yes |
| 1704.583s | -1.000 | -.187 | .601s | yes |
| 1790.305s | -.196 | -.265 | .606s | yes |
| 2243.985s | +1.000 | -.120 | .198s | yes |

Four of five exit events have new negative planner and PID demand. The original
inherited base controller can retain `handoff_active` after its original
positive-PID coast-release predicate no longer holds; the base method uses
`self.handoff_active or low_demand` unconditionally while release is in progress.
This makes a stale latch a plausible cause of those four releases. **No native
controller internal-state message was recorded, so causation is not confirmed.**

## Experimental candidate A: cancel stale hydraulic-to-coast handoff

If normal hydraulic braking is already active, session allowed, radar lead fresh
(<= 0.15s), selected lead relevant, current plan fresh, planner brake
>=0.08m/s², and downstream PID <=-0.05m/s², cancel the inherited
`handoff_active`, `handoff_counter`, and `release_counter` before the
base hydraulic update. This does **not** request additional pressure, create a
new brake state, or bypass the driver's override. It cannot fix releases due to
unrelated base safety-hard-release conditions. Its external-signal screening
matched four of the five gaps, but the relevant internal latch is **unknown**.

## Experimental candidate B: prepare neutral coast earlier

For a trusted, fresh radar lead, ego >=8.0m/s, closing>=3.0m/s, TTC<=12s,
distance between max(8m, 0.6*vEgo) and 100m, after three consecutive 20Hz
confirmations, with no urgent braking, curve braking, stop/hold or release:
hold `0x273 ACC_CMD` at vEgo and set both IS_ACCEL and IS_DECEL bits to 0
while `0x271` is READY. This only suppresses positive framing/offset, does
not generate a negative speed target or force a hydraulic brake, and never
prevents base hydraulic entry.

The retrospective Nov 5 20Hz output screen using logged lead visibility in
place of internal relevant-trust (a proxy, NOT exact native replay) finds 21
READY+IS_ACCEL frames over five segments; marked weak braking --6 would
qualify first near t475.903s, ~0.65s before its t476.554s hydraulic entry.
This is not a demonstration of improved following distance.

## Regression and validation gates

1. Native Python compile and import of the modified controller + policy on the
   OnePlus 3T; run `python3 -m unittest discover -s
   selfdrive/car/dnga/tests -p test_longitudinal_r1_policy.py`.
2. Full controller same-input V5.0 vs R1 replay using native 20Hz longitudinal
   events and CarState, testing 39 segments. Must verify release cause and
   confirm unchanged urgent/low-speed/hold decisions, no new brake entry latency
   or response delay, and no positive acceleration during hydraulic braking.
3. Preserve the proven **0x275+0x08C veto**, since zero 0x08C alone cannot
   establish regen-free operation; no modification to 0x273 DECEL framing
   during hydraulic braking.
4. Require independently verified brake-response / pressure data and closed-
   course supervised physical safety verification **before** any engagement
   with changed longitudinal actuation. Not for public-road experiment.

**Status:** Unit tests of pure R1 predicates pass locally. The integrated
controller has not been physically tested, and full native controller
regression has not been run. The experimental feature remains OFF.
