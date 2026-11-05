# V5.0 OP longitudinal — November 5 evidence and hold points

**Branch purpose:** non-actuating diagnosis only. This branch starts at V5.0 commit `ce841812a5b17a6bdcac9b4e9deb5235f06978b9`. It deliberately does not change steering, 0x271/0x273 TX, CAN parser, safety, thresholds or brake timers.

## Reproduced events

These values are calculated from decoded, time-aligned OP and OEM 20-Hz CAN pairs; quantities are engineering-inferred, not validated actuator pressure or wheel torque.

| Afternoon route time | Observed OP brake request | Measured response | Notes |
|---|---|---|---|
| 476.554–478.253 s (segment --6) | decoded magnitude ~0.06 -> 0.42 | speed ~29.04 -> 28.37 km/h; median aEgo ~-0.006m/s²; friction 0 -> 28 | sustained decel below -0.15 m/s² first appears ~1.55 s after entry; driver brake soon after |
| 848.252–850.453 s | decoded magnitude ~0.06 -> 0.07 (peak ~0.23) | speed increases ~0.98 km/h; near-neutral median aEgo; friction peak ~29 | separate candidate weak-response context; examine lead/planner before calling a fault |
| 1000.513–1002.313 s (segment --14) | decoded magnitude ~0.06 -> 0.15 (peak ~0.18) | speed ~40.33 -> 40.50 km/h; RPM ~1371 -> peak 2806; friction 0 -> 66 | OP IS_ACCEL=0, IS_DECEL=1; driver CANCEL at ~1002.33s |

**Do not confuse ramped command magnitude with measured hydraulic pressure.** In the weak-braking --6 case, an initially small encoded request gradually increased, but the log does not establish whether the ramp, ECU acceptance, powertrain mode, or other factors caused slow deceleration.

The standalone offline audit of paired CAN samples identifies 31 approximately contiguous sampled OP BRAKING episodes (35 episodes in complete native 0x271 TX stream), 3 weak-response candidates and 5 short re-entries. Sample-count differences result from pairing/sampling; prefer native 0x271 for definitive episode counts.

## Interpretation

1. Current V5.0 already commands IS_ACCEL=0 under 0x271 BRAKING. The segment-14 RPM increase is not evidence of transmitted 0x273 positive-accel request.
2. 0x275 word 0 closely follows the factory camera braking request, **not** independently measured OP hydraulic actuation; 0x08C responds to OP-only braking, but it is not calibrated brake force.
3. Upstream PID positive while planner negative may occur while final OP 0x273 remains braking/neutral. Do not treat positive PID as actual OP propulsion unless the transmitted CAN command supports it.
4. OEM camera source-2 0x271 entry may precede OP source-0 entry by 0.76–3.03s in specific approaches. This is a difference in camera/request thresholds, **not** proof stock ACC would apply brakes sooner or harder while OP is active.
5. Existing final propulsion interlock uses 0x275 and 0x08C conservatively. **Do not weaken it** from the observed correlations.

## Additional lead/planner sequence (backward-only historical alignment)

- Segment **--6**: at route t=476.55s, lead ~37.85m ahead with vRel~-4.64m/s. At 478.16s, lead ~28.42m, vRel~-5.8m/s. The planner's first-point acceleration changed from approximately -0.013m/s² to -0.429m/s² over that interval; the user then intervened. OP was **already BRAKING**; inadequate physical deceleration and the gradually increasing magnitude must be evaluated as separate effects.
- Segment **--14**: around 1000.46s, lead ~21.1m, closing by ~1.44m/s, and first-point planner acceleration was still **positive ~+0.07m/s²** when hydraulic braking began. At 1001.30s the lead had increased to ~23.6m and was closing only ~0.27m/s; by 1002.20s it was opening at ~+0.70m/s and distance ~26.8m, while the planner still asked for modest deceleration and OP continued BRAKING. This offers a **specific coast-release hypothesis** for offline shadow tests (lead pulling away while braking and driver/PID seeking acceleration), NOT authority to release brakes in the vehicle. The measured aEgo-based release gate blocks handoff when aEgo is positive, which coincides with the segment-14 RPM rise. The physical/ECU meaning of RPM remains unresolved.

Avoid simplistic release-by-vRel: lead selection changes, radar noise, curves, and brake-intervention conditions can make such a rule unsafe. Any candidate must preserve urgent/reentry exceptions and be counterfactually replayed across all episodes first.

## Candidate cause hypotheses requiring discrimination

- Slow 0x271 hydraulic entry or ramp versus closing-lead trajectory.
- Hybrid drive torque lingering after OP changes from positive propulsion into hydraulic BRAKING.
- ECU arbitration / bus-2 OEM READY versus OP bus-0 BRAKING and resulting physical brake response.
- Re-entry following V5.0's coast-first release; changing hysteresis alone risks later braking.

## Additional counterfactual: coast-release shadow scan

An offline-only backwards-timestamp-aligned scan was applied to the prior decoded November 5 logs. No CAN commands were changed. The *hypothetical* release candidate required all of: existing OP 0x271 BRAKING, vEgo >=10m/s, selected radar lead valid and dRel >=20m, vRel >=-0.5m/s, first-point planner accel >=-0.20m/s², downstream requested accel >=+0.08m/s², measured aEgo >=-0.05m/s², and three successive approximately 50ms qualifying samples.

**Result:** 12 qualifying samples, **all** in marked afternoon segment --14, from route t~1001.763 to 1002.313s. No other recorded segment qualified under this narrow shadow rule. The candidate appears only as the lead moves away while OP hydraulic brake state remains active, in the rising-RPM episode.

This supports investigating an overly restrictive `COAST_HANDOFF_AEGO_MAX=-0.05` gate for *nonclosing leads*, but **does not approve implementing the hypothetical release rule**. Such a release could be hazardous if the lead brakes again or radar changes selection, and the original command's physical effect is unverified. No V5.0 moving-release or braking output was changed.

## Required before any actuation change

- Create same-input V5.0 baseline versus hypothetical earlier-entry/ramp shadow outputs without transmitting CAN.
- Validate bus, payload byte, pump and magnitude semantics with known-good stock ACC captures including identical speed/lead/road conditions.
- Obtain independently verified brake force/pressure or equivalent physical response; 0x275/0x08C alone are insufficient.
- Closed-course, supervised validation before any engagement; keep physical driver braking available.

**HOLD:** this audit does not establish safe positive braking authority, and is not a vehicle-ready longitudinal fix. Avoid experimental OP longitudinal on public roads while the weak-braking and rev events remain unresolved.
