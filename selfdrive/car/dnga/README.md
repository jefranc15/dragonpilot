# DNGA Yaris Cross HEV

V4.2 keeps the cleaned V4 file structure while returning longitudinal behavior
toward the last known-good pre-feedback V3.3R baseline
(`5a5ba4c9cb13b6135dc01fc7e3ce5815f4eb35d3`).

The 0x275-family signals are read-only physical feedback. They do not own ACC
engagement, the SET/set-speed display, LKAS state, steering authority, or the
hydraulic brake request.

## Structure

- `carcontroller.py`: steering limits, CAN scheduling, HUD, and actuator feedback.
- `longitudinal.py`: lead qualification, desired-speed shaping, hydraulic braking,
  stop-and-go, and brake-to-propulsion handoff.
- `dnga_hybrid_feedback.py`: read-only decoding/freshness/agreement for 0x275,
  0x2C9, 0x12A, 0x125, and 0x08C.
- `carstate.py`: vehicle state, buttons, ACC MAIN/SET/RES, and LKAS latches.
- `interface.py`: car parameters plus passive bus-1 HEV and bus-2 stock-camera observation.
- `dngacan.py`: outgoing 0x1D0/0x271/0x273/0x274 encoding and checksums.
- `values.py`: vehicle identification and active control tuning.

The authoritative DBC is `opendbc/dnga_hev.dbc`.

## V4.2 control contract

ACC/session presentation follows openpilot engagement plus the CarState
ACC MAIN/SET/RES latch. CANCEL and the brake pedal normally clear that latch in
CarState; the controller-level cancel/pedal/gas gates can suspend actuation
without being allowed to rewrite the 0x273 session on their own. Driver
accelerator override therefore preserves the visible ACC session. HEV feedback
cannot clear or latch the cruise session and cannot change LKAS/HUD engagement
state.

Normal 0x273 mode follows the stock-observed V3.3 state table:

- disabled: IS_ACCEL=0, IS_DECEL=0
- normal 0x01, including a lowered lead-deceleration target: IS_ACCEL=1, IS_DECEL=0
- moving brake 0x21 / crawl 0x31: IS_ACCEL=0, IS_DECEL=1
- standstill hold 0x30: IS_ACCEL=1, IS_DECEL=1

For a trusted lead, V4.2 restores V3.3-style below-current-speed 0x273 shaping
before and after hydraulic braking. Generic no-lead negative targets remain
disabled. Positive propulsion still requires planner/PID agreement.

Hydraulic braking remains planner-primary. A fresh checksum-valid stock-camera
0x271/0x273 braking pair is direct brake-only evidence and is not vetoed by a
temporarily positive downstream PID. The geometry-only fallback requires a
trusted closing lead plus matching negative planner intent. V4.1's stronger
0.24 final-crawl floor is retained for stop completion.

## V4.3 curve and set-speed corrections

The software DNGA cruise latch now sets `CarParams.pcmCruise = True`. This does
not hand longitudinal actuation back to the stock PCM; openpilot longitudinal
control remains enabled. It tells `controlsd` that `carState.cruiseState.speed`
is the authoritative setpoint. The cluster SET speed and planner `vCruise`
therefore remain the same value during short and long SET/RES presses. A press
that already produced a held 5 km/h step no longer adds an extra 1 km/h on
release.

No-lead negative 0x273 targets remain opt-in. V4.3 allows them only when:

- `longitudinalPlanSource == turn`, using a bounded 1.4 s lookahead from the
  vision-turn acceleration trajectory; or
- the selected planner source is normal cruise and actual speed is above the
  user SET speed.

Curve anticipation uses 60% of the most negative acceleration in the first
~1.4 s of the plan and caps it at -0.35 m/s². V4.3 logs showed that the normal
0x273 below-vEgo path was limited to roughly 1 km/h below actual speed at
50-60 km/h and produced almost no measured deceleration. V4.3.1 therefore
restores the historical curve-only 0x21 supplement, but only when the planner
is explicitly source=turn, no lead is controlling, speed is above 5.6 m/s,
the car is at least 0.70 m/s above visionTurnSpeed, and the anticipated turn
request is at least 0.12 m/s². Entry requires three 20 Hz confirmations.
Authority uses the old bounded curve envelope (about 0.17 at 8 m/s, 0.23 at
15 m/s, and no more than 0.30 m/s²). It cannot arm stop-and-go and releases
when the turn source clears or the vehicle reaches the turn target.

## V4.3.2 lead-brake and low-speed cruise corrections

September 25 V4.3.1 logs showed three separate issues that are corrected here
without changing the 0x275-family observer:

- Lead braking could start late because DNGA waited for the first planner accel
  points to become negative even when the same MPC trajectory already scheduled
  meaningful decel about 1.4 s ahead. V4.3.2 adds a bounded lead-only lookahead
  for smooth 0x273 target shaping and normal hydraulic entry. It does not feed
  urgent/emergency classification.
- A checksum-valid stock-camera 0x271+0x273 brake pair is accepted as direct
  brake evidence earlier (up to 20 m/s and 45 m) and no longer needs an extra
  radar closing-speed threshold. That direct OEM request may bypass the initial
  brake-entry settle timer.
- Predictive stop-guard authority no longer jumps directly to the old 0.87
  ceiling. Stock-backed braking follows the validated stock request with a
  small margin; geometry-only stop guarding uses a speed-dependent ceiling.
- Once the stopped-lead creep path arms, it stays latched through the 8-12 m
  crawl window instead of releasing for about 0.5 s and immediately re-entering
  0x31 when the planner briefly asks for positive acceleration.
- Near the user SET speed, if the cruise planner is only slightly positive while
  the downstream speed PID persistently asks for decel, a three-sample
  hysteresis applies a tiny coast-biased 0x273 target. This addresses the
  repeatable low-speed HEV engine-rev fault without assigning any new authority
  to 0x275 feedback.

The near-set coast path is intentionally small (-0.08 m/s² synthetic decel
before the existing powertrain target cap) and exits when PID asks for positive
acceleration again. The normal 0x275 brake-to-propulsion readiness gate still
applies on the return to propulsion.

## V4.3.4 stock-brake continuity corrections

October 2 V4.3.3 logs showed that CAN scheduling and the 1.2 s release-pump
framing were healthy, but the hydraulic arbitration could still create repeated
brake-release-brake cycles while the factory camera remained continuously in
0x21/DECEL.

- An active stock/predictive brake guard now cancels the positive-PID hydraulic
  handoff and resets its release counter. The guard therefore wins while valid
  factory braking evidence remains present.
- Soft hydraulic release stays at BRAKE_MIN until RELEASE_CONFIRM_COUNT is
  satisfied, then moves atomically into staged release. This removes the
  one-frame READY/pump=0 hole seen between 0x21 and the -0.4 release-pump phase.
- During staged release, 0x273 remains DECEL while the read-only HEV observer
  still reports a pending physical brake-to-propulsion handoff. Once the
  0x275-family feedback is clean, the normal READY/ACCEL mode may resume.
- Direct stock braking authority is admitted out to 70 m and up to +1.0 m/s
  apparent lead opening. Once active, a +1.5 m/s continuation threshold and
  three-sample debounce prevent brief radar vRel noise from dropping the guard.
  The stock camera leaving the validated BRAKING/DECEL pair still releases the
  guard immediately.
- The validated stock pair is now brake evidence rather than a copied brake
  magnitude during normal following. Stock-backed entry is capped at 0.12,
  then rises at 0.012 per 20 Hz update toward the larger of a 40% OEM-reference
  request and a closing-energy request. This preserves earlier/smoother braking
  instead of reproducing the factory controller's late, hard ramp.
- If closing becomes urgent/critical, the rise rate increases to 0.03 per update
  and the validated OEM request may contribute up to the existing 0.45 urgent
  ceiling. The normal smooth path therefore does not reduce emergency authority.

The existing HANDOFF_COUNT, RELEASE_CONFIRM_COUNT, RELEASE_PUMP_FRAMES,
REENTRY_BLOCK_FRAMES, HYBRID_READY_COUNT, and brake scaling are unchanged.

## HEV feedback scope

The HEV observer is used only at a deceleration-to-propulsion boundary. A prior
negative target, hydraulic brake, or SNG release marks a handoff pending. Positive
desired-speed buildup remains at zero until three consecutive 20 Hz samples
confirm:

- all five HEV channels are fresh and mutually consistent;
- 0x275 brake request and 0x08C friction are clear; and
- 0x275/0x2C9/0x12A/0x125 show that strong negative torque has faded.

That three-sample (~0.15 s) confirmation is signal debounce, not a blind release
timer. Stale or disagreeing feedback never disables ACC, LKAS, steering, or
hydraulic braking; it only prevents a pending brake-to-propulsion transition
from building a positive speed target.

The stock-observed 1.20 s 0x271 FC/04/C8 release sequence is retained as protocol
framing, but the timer itself no longer blocks 0x273 normal mode or positive
target buildup after physical HEV feedback proves the handoff ready.

The active Panda safety policy is unchanged.

## Deliberately retained safeguards

Lead qualification, brake entry/rate limiting, measured-deceleration governor,
low-speed ECU wake, stopped-lead confirmation, 0x21 -> 0x31 -> 0x30 stop states,
lead-release hold, and overshoot recovery remain. These are independent of the
0x275 observer and should be changed only from road-log evidence.

V4.0's differential equality harness was removed from V4.2 because V4.2
intentionally changes longitudinal behavior. The original harness remains on
the V4.0 branch for auditing the refactor itself. Native-device syntax checks,
Panda checks, and controlled vehicle testing are required before relying on
V4.2 stop completion.
