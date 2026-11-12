# v6.0 R2 — passive stock 0x273 pre-brake lead audit (2026-11-09)

**Read-only observational test, not HIL or OP-engaged replay.**
R2 remains disabled for on-road CAN actuation:
`V6_R2_ACTIVE_CAN = False`.

## What was tested

Records: 8 November 8, 2026 stock camera ACC segments (0–7),
and four older rlogs labeled 127–130. Thirty-three-bit? No: 8-byte
CAN frames decoded on bus 2 at 20 Hz, matched to timestamped
`carState.vEgo` through the archived offline decoder.

- **28** transitions from camera `0x271 READY (0x01)` to
  `BRAKING (0x21)` while factory ACC was enabled.
- Immediately before all 28 BRAKING entries, the factory `0x273 ACC_CMD`
  was at least **0.5 km/h below measured vEgo** with `0x271` in READY.
- On the immediate pre-brake sample, **26/28** had
  `IS_ACCEL=1, IS_DECEL=0`; the other **2/28** had
  `IS_ACCEL=0, IS_DECEL=1`.
- The lower-than-vEgo target was common even without an imminent brake:
  approximately **84.2% of 6,232** moving, factory-enabled
  `0x271 READY` samples had a gap >= 0.5 km/h.
- The threshold was already present in the preceding 1-second window
  in all 28 events, but the condition is so frequent in READY
  operation that it does **not** uniquely identify impending
  hydraulic brake engagement.

### Interpretation

The OEM can send `ACC_CMD < vEgo` before changing 0x271 to BRAKING.
However the OEM usually keeps `IS_ACCEL=1` during that READY interval,
so a pre-hydraulic `IS_DECEL=1` phase must **not** be asserted to be
stock-identical. The mode-bit names are DBC-level names and should not
be assumed to describe physical regenerative torque.

The current R2 `_build_command` only imposes its newly fitted lower
0x273 speed target when `brake.hydraulic` or the explicit moving
release-pump phase is active. R2 does not currently send a
separate, sustained `ACC_CMD` early-deceleration stage in the
moving-brake state machine.

The earlier factory replay is not a direct efficacy benchmark
for OP control. None of these stock camera captures includes
an *engaged* OP downstream longitudinal PID demand. No physical
dynamics can be computed counterfactually from them.

## Validation decision

Do not enable R2 CAN solely on this observational result.
The current Panda safety hook contains a hardcoded
`controls_allowed = true` and whitelists `0x271`/`0x273`
without validating magnitude, request rate, driver overrides,
engagement state, or fail-safe release. The R2 24-update READY/-0.4
tail and urgent first-frame 0.08 request also remain unvalidated.

Three read-only source-level regression checks added to
`test_v60_integrated_r2.py`:
(1) unauthorized e2e-only negative demand cannot create R2
hydraulic braking or an R2 speed gap; (2) initial R2 hydraulic
command keeps ACCEL off, DECEL on and target below vEgo;
(3) R2 READY release tail keeps ACCEL off and DECEL on.

Next tests must use actual OP-engaged plans/PID and a closed-loop
bench/HIL with independently enforced Panda safety. No live R2
transmission, no CAN safety bypass, no release-tail changes were
made in this audit.
