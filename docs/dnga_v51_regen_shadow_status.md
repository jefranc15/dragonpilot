# v5.1-Regen — diagnostic / shadow-only branch

**Not approved for active vehicle braking.** This branch retains the V5.0 longitudinal controller and Panda safety behavior unchanged.

The R2-V4 concept investigated on recorded DNGA Toyota Yaris Cross HEV logs:
- OpenPilot planner-based early, gentle deceleration (instead of late stock ACC escalation).
- 1216 as an *observational* weak-braking reference, not a cap on stronger braking.
- Coordinated study of 0x271 braking requests and 0x273 target-speed commands.
- Read-only 0x275 / 0x08C brake feedback for handoff diagnostics.
- Stock release framing 1224/-0.4 distinguished from normal READY 200/0.
- Smooth target gap changes, and continuation checks at planner-source transitions.

The offline V4 replay: 38 V4 tests and 48 V3 tests passed, but 251 proposed BRAKING frames differed from V5.0 READY, 546 of 646 braking frames were outside the original genuine-stock ACC calibration domain, and no ECU-response/physical braking validation has been performed. The 2026-11-06 V5.0 drive previously exhibited an unexpected acceleration surge.

**Important:** The shadow code predicts hypothetical commands only and cannot prove safe regenerative response. Neither a simulation passing nor a negative 0x275 signal guarantees absence of positive wheel torque.

For research, review the offline V4 package from the related ChatGPT session. Do not interpret the branch name as a road-ready firmware release. A hardware-in-the-loop or instrumented closed-course validation plan, with independent driver override and brake authority, is necessary before live longitudinal actuation.

## Safe installation check

On the OnePlus 3T, collect passive rlogs using the existing verified OEM ACC path. Do not enable experimental OpenPilot longitudinal braking to test this concept.
