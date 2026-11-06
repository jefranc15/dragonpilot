# V5.0 no-lead curve release / HEV rev / acceleration forensic update

**2026-11-06. OFF-CAR diagnostic evidence only. No production longitudinal/Panda change.**

Based on original Nov 6 `2026-11-06--02-08-06` segments --85/--86/--87,
Nov 5 rlogs, and uploaded Nov 5/6 R2 dual native replay results.
Full time-aligned report and plots are in conversation artifact
`V5.0_R2_hybrid_surge_root_cause_audit.zip`.

## What is established

1. **Release:** V5.0 curve `low_demand` can trigger when the
   curve-speed error is <=0.20 m/s, planner brake <0.06 m/s², OR
   planner source stops being `turn`. In the severe --87 event the
   release counter first rises while still on `turn` because the turn
   speed target was already met. Source flicker sustains the count.
   After 10 counts, real source-0 0x271 BRAKING->READY occurs at
   segment-relative ~44.274 s; off-car replay clear is one 20 Hz tick
   later at ~44.325 s.
2. **Brake-to-coast transition:** Source-0 0x273 IS_DECEL=1 during
   hydraulic 0x271 and goes to IS_DECEL=0 at release. IS_ACCEL stays 0.
   Active ACC/session and speed target `ACC_CMD≈vEgo` persist
   during READY. These bits are not Techstream-validated guarantees of
   zero wheel torque. Physical friction 0x08C then falls from raw 63
   to 0 at ~44.577 s, at the same sample where measured aEgo turns
   positive.
3. **Motion:** No driver gas raw 0x277 and no real lead
   (`longitudinalPlan.hasLead=0`, `radar.leadOne.status=false`).
   CarState aEgo peaks **+1.293 m/s²** at ~45.285 s, even though
   downstream accel demand is ~**-2.07 m/s²**. Speed rises
   ~**+6.71 km/h in 2 seconds** after release. Engine 0x037 peaks
   **3,047 RPM** raw and approaches 0 by ~48.915 s while speed
   has continued rising. Valid `liveLocationKalman.accelerationCalibrated`
   shows a corroborating +0.99 m/s² median during 44.7-45.5s;
   it is fused and may share wheel-speed inputs.
4. **Why OP does not counter the rise:** Curve hydraulic re-entry
   requires `plan.curve_active`, speed error >=0.70 m/s,
   **planner brake >=0.12 m/s²**, and no reentry block (40
   100 Hz frames). During the surge with source `turn`, planner
   deceleration quickly declines below 0.12 even as downstream
   demand reaches ~-2.3. Then the source switches to `cruise`.
   OP remains READY/neutral and supplies **no further braking**.
5. **Hybrid feedback:** 0x275 raw word2 peaks ~1245, 0x2C9 raw
   field peaks ~1239 and 0x12A/0x125 fields reach ~101/100.
   All five raw feedback IDs pass V5.0 additive checksums. The
   fields track engine RPM **even while the car is decelerating**.
   The V5.0 code explicitly labels their physical signs/engineering
   meaning as **NOT validated**; these values do not prove positive
   MG2 or petrol-engine wheel torque.
6. **Camera vs OP:** V5.0 source `dnga_fwd_hook` drops factory
   bus-2 0x271/0x273 forwarding to bus 0. In ~1.6 s around the
   severe release, **32/32 OP 0x271 and 32/32 OP 0x273 source-0
   TX bytes match corresponding Panda source-128 transmit echoes**.
   The factory camera bus-2 0x273 is separate and has IS_ACCEL=1.
   Echo confirms the Panda TX path, **not** ECU receipt or flashed
   firmware checksum. Panda log mode: Toyota safety param 1.
7. **Cross-case specificity:** Six no-lead/no-gas curve-related
   hydraulic releases exist in the three Nov 6 segments. Only the
   severe --87 release is followed by a large acceleration surge:
   other positive peak acceleration <=+0.324 m/s², friction before
   release <=22 raw, 0x2C9 raw peak <=520. Severe: +1.293,
   friction 63 and 0x2C9 peak 1239. A conservative 20 Hz shadow
   screen detects just **one 1.631-second / 33-sample** event across
   BOTH Nov 5 and Nov 6 replay traces.

## Physical-cause diagnosis: ranked, not yet proved

The **controller-side cause of uncaught acceleration is established**:
0x271 brake release, actual friction unwind and loss of 0x273 decel,
plus curve-only reentry never rearming because the planner demand drops
below entry threshold while the PID remains deeply negative.

The **physical positive-force source is unresolved**. Leading hypotheses:
- residual HEV traction or engine/motor-regenerative handoff exposed
  when friction/regen releases; possible despite zero pedal and OP
  IS_ACCEL=0; the hybrid raw fields do not identify torque direction;
- grade/downhill after an uphill crest, alone or with HEV torque;
- enabled OP 0x273 target tracking current ego speed even when both
  accel/decel mode bits are zero (ECU interpretation unknown).

Gravity alone would need **at least ~13.3% downhill slope** at the
+1.293 m/s² measured peak, *ignoring* drag/rolling losses. If the
road was still ascending at that instant, real positive traction torque
must have existed. The recorded GPS altitude, NED velocity and
orientation/pitch fields have **valid=false** and cannot decide slope.

Engine RPM rising while BRAKING could represent hybrid regenerative or
engine braking and is not proof the ICE was propelling the car.
For manufacturer context see
https://toyota.jp/faq/show/6790.html (Toyota hybrid engine-braking
versus regen depends on battery/hybrid state), which does not decode
this Yaris Cross's actual torque channels.

## R2 diagnostic additions and recommended next actions

- Added pure OFF-CAR `tests/r2_shadow_surge_screen.py` and 3 regression
  cases in `test_longitudinal_r2_shadow_surge.py`. It screens
  `r2_20hz_state_trace.csv`: READY, no planner lead, no driver
  pedal, IS_ACCEL=0, measured aEgo >+0.5 and downstream <-0.5.
  **Note:** CarController's generic `lead` argument is HUD
  `leadVisible`; do not treat it as an actual radar lead.
- Next instrumentation should verify actual *flashed* Panda forwarding
  and obtain signed MG2/MG1 requested/actual torque, ICE torque,
  HV battery current/SOC, actual regen demand, independent grade,
  and accurate vehicle speed on a common timeline.
- Do not treat this evidence as permission to prolong friction braking,
  deepen 0x273 negative targets, bypass Panda safety, or enable
  public-road R2 longitudinal. A hypothetical acceleration-recovery
  control should first be evaluated in an isolated **shadow replay**,
  with disengagement, lead-entry, stale-plan, curve, low-speed and
  driver-pedal override cases, then independently validated in a
  controlled environment.

**No drive/steer/brake/safety file is altered on this R2 branch.**
