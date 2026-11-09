# V5.2 R2 direct 1216 moving-brake development (NOT road-ready)

Branch: `V5.2-R2-1216-direct-dev` (based on `V5.2-Test`).
This branch intentionally **does not enable direct CAN actuation by default**.
`carcontroller.py` sets `R2_V4_DIRECT_CAN_ENABLED = False` until ECU
response, safety and device-level validation have independently passed.
Do not interpret successful pure-Python CI as permission for engaged driving.

## Source evidence from the supplied Drive_log_analysis.txt

- Corrected November 5 V5.0 replay: all five short BRAKING→READY→BRAKING
  events had mild negative planner requests but **positive downstream PID
  acceleration** at release. Four were ~0.60 s apart, consistent with a
  0.40 s reentry guard followed by ~0.20 s entry confirmation. The R1
  stale-handoff predicate did not match any of them.
- November 6 no-lead uphill curve, segment --87: while OpenPilot was in
  READY/neutral after a curve brake release, measured acceleration reached
  +1.293 m/s², engine RPM peaked at 3,047, and no driver gas or positive OP
  IS_ACCEL was recorded. This physical outcome is **not explained** by the
  acceleration flag alone.
- Subsequent genuine factory stock ACC D sessions: 988 factory BRAKING
  frames, 27 entries, and all 988 had negative first-word 0x275. The first
  BRAKING frame was preceded by READY and already had negative 0x275.
  Of those frames, friction 0x08C was positive only ~50.2%.
- Observed stock command/feedback association:
    1216 (0x04C0, pump -0.4) ~ -163 first-word 0x275 (gentle);
    1208 ~ -317; 1199 ~ -512; 1190 ~ -696.
  The relationship `0x275 ≈ -20.48 * (200 - (0x271 magnitude & 0xff))`
  had correlation 0.9994 in the stock sample. It is NOT calibrated physical
  motor torque, brake pressure, or regenerated energy.
- Twenty-five stock BRAKING→READY releases began with magnitude 1224
  (0x04C8), pump -0.4. The ordinary READY value 200 / pump 0 is a
  distinct state. The factory 0x273 target often was already below vEgo
  **before** the first BRAKING frame, so matching only the first magnitude
  is not a complete stock-ACC clone.
- The stock 0x273-command fit has ~0.20 km/h MAE on 949 held-out factory
  samples, but this is a command-field fit, not vehicle response testing.
  Recorded speed coverage only reaches ~79 km/h in that sample.
- Additional factory pump settings -0.5 and -0.6 exist, so the earlier
  -0.4/-0.5 candidate doesn't cover the complete factory brake envelope.

## What the code actually changes

Previously in V5.2-Test:
  `V5.0 update -> V5.0 interlock -> R2 experimental *combiner* -> CAN`.
The combiner always kept a stronger V5.0 moving-brake magnitude as its floor
and retained V5.0's hydraulics/coast/reentry arbitration.

This branch creates `r2_v4_direct_authority.py` and removes that combiner
from `carcontroller.py`. If explicitly enabled **after independent
validation**, the moving command producer uses the R2 V4 planner proposal as
the authority for BRAKING vs READY, without requiring a preceding V5.0
coast/handoff and without retaining a normal V5.0 braking strength floor.

Normal entry is BRAKING 0x21 / magnitude 1216 / pump -0.4 even when the
unmodified base controller would be coasting or accelerating. Normal strength
is stepped 0.02 per 20 Hz update, urgent up to 0.05. R2 provides the below
vEgo 0x273 command target and IS_DECEL. The observer provides a temporary
source bridge, gap target slew, READY/1224 release and a three-fresh-feedback
0x275/0x08C acceleration eligibility proposal.

V5.0 still computes planner/subscriber data, retains its *positive*
propulsion speed shaping, and is the deliberately retained source of SNG,
HOLD/CREEPING and low-speed entry. A genuine urgent V5.0 brake may also take
authority so this architecture does not weaken an urgent request merely
because R2 reports READY. The low-speed handoff is not yet HIL-validated.
This is a moving-brake-policy replacement, not a deletion of every V5.0
longitudinal subsystem.

The new final R2 brake/propulsion interlock is **after** R2 selection: it
vetoes positive ACCEL during BRAKING/release and on stale/active brake
feedback without destroying the coordinated below-speed ACC target.

When the CAN gate is off, the original V5.0 command is sent, and R2 is
observation-only. The older V5.2 experimental combiner is not used.

## Outstanding safety blockers

1. Actual ECU behavior of 0x271 1216 with below-speed 0x273 has never been
   measured for this exact implementation. Existing 0x275 correlation is a
   factory *request* correlation, not actual regenerated torque.
2. A missing expected 0x275 is reported as a fault but cannot safely be
   translated into an automatic brake-release/disengagement policy yet.
   Braking effectiveness and independent friction measurements are needed.
3. Source bridging, target-gap ramp and 24-update READY release are replay
   hypotheses, not validated closed-loop controls. Long-duration engine
   revving during brake release has occurred in earlier versions.
4. The underlying custom Panda safety configuration has been permissive.
   Safety model, driver brake/CANCEL, ECU command forwarding, actual
   control-suppression and camera calibration need separate verification.
5. Current CI runs pure R2 logic and syntax checks only; it does **not**
   run complete CarController with native cereal/packer, physical brake
   response or closed-loop hardware-in-the-loop.
6. Low-speed/SNG handoffs and high-speed emergency requests are not
   certified for this new authority.

## Non-actuating checkout and review on OnePlus 3T

Run only when parked; does not switch installed Dragonpilot:

```sh
cd /data/openpilot
git fetch origin refs/heads/V5.2-R2-1216-direct-dev:refs/remotes/origin/V5.2-R2-1216-direct-dev
git diff --stat V5.2-Test..origin/V5.2-R2-1216-direct-dev
git show origin/V5.2-R2-1216-direct-dev:docs/dnga_v52_r2_direct_1216.md
```

See GitHub Actions for unit-test status; no CAN messages were transmitted
during these offline tests. Keep direct CAN gating disabled until reviewed
and tested against independent ECU and brake-response measurements.
