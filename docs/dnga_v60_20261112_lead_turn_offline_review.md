# DNGA v6.0 `2e62845` — read-only lead/turn follow-up (Nov 12, 2026)

Source: user's `v6.0-2e62845.zip` (29 rlog/qlog segment pairs), plus
the earlier Nov 10–11 `v6.0(1).zip` (34 usable segments).
**This is a diagnosis and non-actuating advisory prototype, NOT a deployed
braking-control change.** Regen/friction share and ECU torque are not decoded.

## 1. Weak lead braking, marked segment 40

| Step | Route-relative s | Relative to first BRAKING |
|---|---:|---:|
| Radar first sees lead intermittently | 2406.216 | −1.122 s |
| Lead becomes continuously visible (approx.) | 2406.760 | −0.578 s |
| 13-point planner lookahead minimum below −0.05 m/s² | 2407.067 | −0.271 s |
| First `0x271 BRAKING` 0.08 (1216) | 2407.338 | 0 |
| Requested normalized strength >=0.20 | 2408.431 | +1.093 s |
| `0x275` raw brake request changes to −512 | approx. 2408.635 | +1.297 s |
| Measured acceleration <= −0.50 m/s² | 2408.956 | +1.618 s |
| Requested normalized strength >=0.45 | 2410.122 | +2.784 s |
| Driver brake/disengagement | 2410.989 | +3.651 s |

Early radar detection was present in only 5/13 initial samples and the
planner near-term acceleration initially remained positive. The gradual
normal moving brake-strength/ACC-speed-gap rise compounded that delay.
The vehicle's realized deceleration remained small through much of the
initial requested ramp. After the command reached the 0.45 cap, measured
acceleration remained near −1.0 m/s² while downstream PID requests
approached −2.0 m/s². RPM rose toward ~2879; transmitted `0x273`
remained in deceleration mode during BRAKING. These observations do not
establish the regen/friction contribution or prove that changing the
normal 0x271 ramp would be safe/effective.

**Conclusion:** there are distinct observable radar, planner, command
ramp and powertrain-response delays. The 80-frame release does not cause
this event, since the driver disengaged while braking was active. Do not
increase the 0x271 request/cap or shorten the lead gate without physical
brake-response calibration and independently verified radar identity.

## 2. Turn selected but no braking, marked segment 42

- Correctly decoded `visionTurnSpeed` reduced from approximately 71.8
  to 70.5 km/h with ego about 73.7 km/h; one later turn sample requested
  66.2 km/h. The turn source flickered with cruise; the immediate planner
  acceleration remained positive during most turn-selected updates.
- The negative 13-point future acceleration minimum varied around
  −0.05 to −0.13 m/s². Current R2 requires 3 consecutive meaningful
  curve samples and `plan.brake >= 0.12`, which was not met. No
  `0x271 BRAKING` was sent before driver intervention.
- **Correction to previous comparison:** the unmarked Nov 12 segment
  with 40 `BRAKING` samples and `source=turn` also had a radar lead on
  every such sample. It is NOT a confirmed lead-independent turn-brake
  case. The earlier Nov 10 curve-chatter event was a no-lead event
  around 35 km/h.
- Across the 29 new and 34 earlier usable segments, only the marked
  segment 42 showed engaged high-speed (>=54 km/h), no-lead turn-selected
  samples. The false-trigger rate at high speed therefore cannot be
  estimated from these logs.

## 3. Read-only advisory hypothesis

`selfdrive/car/dnga/tests/v60_offline_turn_lead_review.py` evaluates
a *non-actuating* early-turn notification requiring: ego >=54 km/h,
current no-lead `turn` source, valid turn target >=2 km/h below ego,
a 13-point future minimum <=−0.075 m/s², and 2 matching samples within
0.35 s. It latches to avoid duplicate notifications. The program sends
**NO CAN frames** and is not imported by openpilot's `CarController`.

On the marked turn it produced one advisory at **2545.124 s**, roughly
0.84 seconds before the driver's brake near 2545.96 s. The first brief
turn at 2530, earlier 35 km/h curve chatter, and lead-present "turn"
samples did not qualify. This retrospective match cannot validate
underlying curvature, stopping distance, an ECU response, or false
activations on unrecorded roads. **Do not use the advisory as live
braking authority.**

Usage after exporting an aligned 20 Hz CSV off-device:

```sh
python3 selfdrive/car/dnga/tests/v60_offline_turn_lead_review.py aligned.csv --mode lead
python3 selfdrive/car/dnga/tests/v60_offline_turn_lead_review.py aligned.csv --mode turn
python3 selfdrive/car/dnga/tests/test_v60_offline_turn_lead_review.py -v
```

Required input: `t`, `source`, `has_lead`, `p_amin`, vehicle
`v_kph` and `vision_target_kph` (or `v` and `vision_speed` in m/s),
and optional `driver_brake` and `engaged`. The lead mode additionally
uses `lead_status`, `brake_state`, `brake_norm`, and `a`.

## 4. Decision

**Do not activate a new early-turn braking policy or faster/stronger
lead-brake ramp on `v6.0` yet.** The observed shortfall required driver
braking and requires full closed-loop acceptance/response validation.
The present logs cannot rule out a false high-speed turn trigger.
Preserve current single controller, hybrid interlock, existing ECU
command envelopes, PID gains, stop-and-go, and **80-frame moving
READY/pump release**. Next stage is controlled bench/HIL analysis with
road-view/modeled curvature, validated lead tracking, received/sent
0x271/0x273, brake feedback 0x275/0x08C, and aEgo.
