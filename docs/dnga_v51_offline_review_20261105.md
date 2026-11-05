# DNGA V5.1-dev — offline validation of 2026-11-05 V5.0 logs

**Scope:** offline diagnosis only. No road, ECU, Panda, or hydraulic-actuator validation. Do not use these results to approve public-road OP longitudinal control.

## Code and test observations

- Base logs: V5.0 commit `ce841812a5b17a6bdcac9b4e9deb5235f06978b9`.
- Development branch retains all V5.0 DNGA longitudinal and safety files unchanged.
- V5.0 `LanePlanner.get_d_path()` mutates its argument, so the supposedly E2E-selected `self.path_xyz` is already lane-blended. V5.1-dev uses independent ndarray copies for lane and E2E and translates the camera offset + the user path offset exactly once.
- Verified the local helper bytes against GitHub blob `8965f9bc5c74e8d51231f7c733e1833025ef0ec6`.
- Local offline tests: 8 tests pass including 10,000 deterministic randomized inputs (input immutability, alias prevention, E2E/lanes separation, both offsets exactly once). Additional repository-focused tests validate 5,000 seeded inputs and two specific offset/alias regressions.

## Recorded lateral geometry (snapshot comparison, not complete model replay)

Source: 39 decoded rlog segments from the two November 5 routes, 1,170 model/lane point-zero geometry snapshots, matched within 0.12s against recorded `lateralPlan.dPathPoints[0]` and `carState`.

Filter: both lane probabilities >=0.80; detected width 2.5–4.0m; absolute model and midpoint lateral Y <1m; vehicle speed >=8m/s; valid lateralPlan sample. **455 snapshots** passed.

| Recording | Filtered n | Model right of lane midpoint | Median model-minus-midpoint | Median nominal V5.1 E2E-minus-logged V5.0 target |
| --- | ---: | ---: | ---: | ---: |
| 11-32-49 | 235 | 84.3% | +12.2 cm | +17.4 cm |
| 14-45-36 | 220 | 70.9% | +8.4 cm | +8.9 cm |
| Combined | 455 | 77.8% | +10.5 cm | +13.8 cm |

Right-positive model lateral Y. The nominal V5.1 counterfactual assumes **dpCameraOffset=+4 cm** (internal -0.04m) and **dpPathOffset=0** for all recordings; those values were NOT independently extracted for each logged sample. It compares the V5.1 E2E candidate at first path point to the published V5.0 `dPathPoints[0]`, without solving a new lateral MPC or asserting actual steering response.

**Important:** The corrected independent E2E candidate may lie **farther right** than V5.0's implicitly lane-blended E2E target. Therefore fixing the alias does not establish right-bias correction, and it is not appropriate to enable this E2E branch for road tests based on these offline results alone.

## OP/OEM CAN and feedback cross-check

Using 21,355 previously time-aligned ~20Hz CAN comparison samples, keeping source-2 OEM camera distinct from OP TX source-0 (and excluding echo source 128):

| OP 0x271 BRAKING | OEM 0x271 BRAKING | Frames | 0x275 word0 negative | 0x08C friction nonzero |
| --- | --- | ---: | ---: | ---: |
| No | No | 18,269 | 0.02% | 0.38% |
| No | Yes | 1,141 | 99.65% | 0.0% |
| Yes | No | 1,017 | 0.0% | 82.30% |
| Yes | Yes | 928 | 100% | 52.05% |

This is observational signal correlation, not a validated actuator truth table. **0x275 word 0 is NOT independent verification of OP hydraulic brake actuation.** Preserve existing conservative acceleration veto until its semantics are independently verified; do not use this finding to relax it.

The exact OP sendcan 0x271 stream contains **35** hydraulic BRAKING episodes, and **five** short within-segment brake-release/re-entry gaps: 0.198s (morning --11), 0.606s (morning --3), 0.599s (afternoon --12), and 0.600/0.601s (afternoon --26).

The first approach in afternoon --6 had OP BRAKING from route time **476.554 to 478.253s** (~1.75s), yet speed changed only **29.04 to 28.39 km/h**, median measured `aEgo` ~+0.001m/s². Driver brake intervention followed ~478.30s. This establishes weak observed deceleration under a sent braking state, not why the actuator responded that way.

Afternoon --14 has RPM ~1,371 to ~2,806 while OP BRAKING and `0x273 IS_DECEL=1, IS_ACCEL=0` with negligible speed decrease, followed by driver CANCEL. No OP positive-accel bit was asserted in that event.

OEM camera request entry preceded OP's in three observed approaches by ~0.76–3.03s; these are differing request thresholds, not proof of physical actuator latency or matched stock ACC performance.

## Time-causal planner / controller diagnostic

Backward-only synchronization at selected 20Hz timestamps in afternoon segment `--6`:

| Route t (s) | OP 0x271 | Planner acceleration[0] (m/s²) | CarControl requested acceleration (m/s²) | Measured aEgo (m/s²) | Radar lead dRel (m) |
| ---: | --- | ---: | ---: | ---: | ---: |
| 476.54 | READY | -0.013 | -0.359 | +0.045 | 37.9 |
| 476.75 | BRAKING | -0.047 | -0.073 | -0.005 | 35.1 |
| 477.10 | BRAKING | -0.135 | -1.000 | +0.008 | 35.9 |
| 478.22 | BRAKING | -0.429 | -0.825 | -0.244 | 30.3 |

The controller had requested deceleration before 0x271 BRAKING, and measured deceleration remained weak after BRAKING was commanded. This is a command-versus-response problem for that episode, not simply a missing brake entry. The numbers do not calibrate brake pressure.

In marked segment `--14`, around t=1001.80–1002.25s, the *upstream* CarControl acceleration request was **positive +0.339 to +0.348m/s²** while the planner still asked for **-0.148 to -0.063m/s²**. Nevertheless, OP TX continued `0x271 BRAKING` and `0x273 IS_ACCEL=0`. This confirms controller/planner disagreement during the RPM rise but **does not** show a transmitted positive-propulsion command or prove an ECU cause. Reconcile the disagreement in offline diagnostics before changing any arbitration logic.

## Acceptance gates

**Pass — offline path-construction logic:** No ndarray alias, camera and path translation each applied once, input immutable, deterministic unit/property tests.

**Pass — retrospective diagnostics:** Raw model-versus-lane differences, 35 OP BRAKING episodes and five short re-entries reproduced; OP-only braking / 0x275 / 0x08C correlation quantified.

**Not passed:** production lateral MPC replay from full model messages; exact historical camera/path configuration per frame; accurate physical lane-center measurement; steering dynamics/curvature tracking; brake-pressure authority; ECU brake magnitude semantics; acceptable urgent braking; rev mitigation; Panda safety verification; vehicle or closed-course tests.

**HOLD:** Do not modify longitudinal actuation, increase brake strength, remove 0x275/0x08C acceleration interlocks, or call V5.1 road-ready. Additional logged comparison and safe controlled validation required.

Local analysis outputs from this review are generated from the user archive and not committed to GitHub due to their recording-derived data content.
