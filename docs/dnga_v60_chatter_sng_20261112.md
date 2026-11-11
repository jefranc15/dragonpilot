# DNGA v6.0 R2: November 10–11 2026 chatter / SNG corrections

**Source:** user-supplied `v6.0(1).zip`, 34 readable driving segments / 110 hydraulic episodes.
**Software validation:** isolated regression tests and source review only; not ECU/HIL validation.
**Before branch:** `a9f20d8`. Backup branch: `v6.0-backup-before-chatter-sng-fixes-20261112`.

## Measured failures

- November 10, `16-20-36--4`: five new normal first-brake requests at 0.08 between ~16:25:30 and 16:25:37 as engine RPM rose. At one entry, immediate OP plan acceleration = +0.281 m/s² and downstream PID = −0.103 m/s². A PID supplement of 0.0258 crossed the 0.02 hydraulic-entry threshold despite positive planner acceleration and no negative future plan point.
- 33 observed first brake entries across the archive match a clear positive-plan/negative-PID-only pattern.
- `14-27-51--6`: no-lead `turn`/`cruise` flicker and an anticipatory negative future point as small as roughly −0.054 m/s² generated a normalized brake floor of 0.08. Engine RPM was near zero in much of this chatter.
- `16-20-36--20`: the historical HOLD code resumed after positive planner acceleration and radar lead speed; the recorded lead dRel changes included large jumps and source changes between lead0/lead1.
- `14-27-51--7`: at ~429.4–432.4 s, **58** active 20 Hz hydraulic frames, commanded strength median **0.45**, peak **0.54**, downstream requested acceleration median **−2.23 m/s²**, measured aEgo median **−0.46 m/s²**. The mean instantaneous deceleration discrepancy was about **1.68 m/s²** over those samples, followed by driver braking around 25 km/h. This is an actual command-vs-response shortfall, not a failure of SNG HOLD. Magnitude-to-physical-deceleration mapping remains uncalibrated.

## Changes to R2 moving braking

1. **PID is scaling, not entry authority.** In a trusted lead context, the negative downstream PID contribution can increase genuine planner braking or bounded, closing-gated forward lead-lookahead demand. It can no longer create a hydraulic entry when both sources of planned braking are zero/positive.
2. **Normal moving reentry during READY/pump release:** 20 100 Hz frames (0.20 seconds) of cooldown after hydraulic release; two consecutive 20 Hz qualifying samples (0.10 seconds nominal), with a minimum 0.08 normalized planned/requested brake demand. This applies only inside the existing 80-frame (0.8 s) moving release. A confirmed, authorized urgent or emergency lead with positive planner braking bypasses the normal cooldown immediately. No authority has been added to untrusted lead detections.
3. **Curve turn-selection confirmation:** require three consecutive 20 Hz meaningful turn samples, planner brake >= 0.12, and either a usable positive vision turn speed with vehicle above its entry/release margin, or (when turn speed is absent/zero) an independently meaningful near-term planner deceleration. This preserves turn braking when the legacy schema provides no usable visionTurnSpeed but stops tiny lookahead-only spikes.
4. **Actual moving brake cap/encoder unchanged.** No unverified strength increase was added in response to missed-SNG under-delivery. The existing emergency/urgent first-entry bounds, speed/lead/curve caps, measured acceleration governor, and original 0x271 encoder are retained.

## Historical stop-and-go HOLD validation

The original inherited `_update_stop_hold` remains responsible for HOLD, CREEPING, and SNG release. Before allowing its six-update automatic standstill-lead-resume confirmation to count, the wrapper now additionally requires:
- fresh planner and radar; the same selected lead0/lead1 source while collecting evidence;
- visible status, stopped ego, positive planner demand, and a moving reported lead;
- at least six continuous 20 Hz radar distance observations, cumulative increase >= 0.15 m, mostly nondecreasing samples and no per-update distance jump > 0.50 m.

Unsafe/stale/jittery observations reset the standstill motion evidence; they do not accelerate the release. Manual driver override/cancel still uses the inherited safety-exit path. **There is no true radar track ID in this wrapper; source stability + dRel trend is only a plausibility gate, not confirmed object identity.** Delayed automatic resumption is possible for a very slowly departing lead.

## Tests

- Existing 30 integration tests kept.
- New tests cover positive-planner/negative-PID false braking; valid lead lookahead; closing gate; turn-source flicker and tiny future dips; valid and absent turn-speed scenarios; moving READY pump normal reentry suppression, delayed meaningful normal reentry, immediate authorized urgent reentry; inherited HOLD plus distance jitter, consistent motion, and driver override.
- Pipeline compiles DNGA modules and runs 43 isolated Python regression tests using the actual inherited brake-target, brake encoding, and HOLD method source AST in a controlled fixture.
- **Not** a hardware validation, complete longitudinal closed-loop replay, proof of Panda safety, brake-pressure control, hybrid torque, or collision avoidance.

## Required before further active testing

The missed-SNG under-delivery and simultaneous engine RPM/positive aEgo under commanded deceleration remain unresolved physical-actuation concerns. Examine received vs sent CAN, brake pressure (when available), friction indicator `0x08C`, brake request `0x275`, engine RPM `0x037`, and actual vehicle acceleration. Do not rely on R2 as an emergency collision mitigation system until closed-loop braking performance and override behavior are validated independently. The existing 80-frame release has not been changed here.
