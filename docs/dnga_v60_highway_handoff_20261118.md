# November 17 DNGA HEV highway hard-launch investigation (offline only)

**Do not road-test experimental OP longitudinal until the unexplained post-release
acceleration is resolved using controlled ECU / brake-response validation.**

Analyzed all seven uploaded rlogs (segments 27–33), running exact deployed
controller SHA `2e62845`. The later `ac189649` only adds read-only tests.

## Observations

- Across **18 exits from 0x271 BRAKING**, **13 transitioned to READY/pump
  release** and **5 to DISABLED** (typically driver intervention).
- On segment 27, release at route `t=1664.5916`, ego ~59.2 km/h,
  cruise `SET_SPEED=75` km/h. With raw pedal zero, ego climbed approximately
  **13.9 km/h over three seconds**, peak **+1.89 m/s²** and **3835 RPM**.
- Actual acceleration began **while 0x271 was READY with negative pump
  and 0x273 was DECEL**, then persisted after `IS_ACCEL=0` and
  `IS_DECEL=0`. Downstream PID commanded up to approximately
  **−1.8 m/s²** while aEgo reached +1.9 m/s².
- In this launch, raw `0x275` brake-request was **0** and the
  `0x08C` friction indicator was **0** after initial release.
  The raw 0x275 torque word rose and then fell, but it is *not* a
  calibrated MG/engine torque measurement. Consequently removing the
  hybrid brake-feedback propulsion veto is not a justified fix.
- A separate segment 33, `t=2003.6–2006.7`, had neutral outgoing
  mode bits, downstream PID +2 m/s², measured deceleration roughly
  −0.6 to −0.7 m/s² and engine RPM up to 3259. Raw 0x275 brake-request
  around **−512** makes existing brake-to-propulsion veto relevant
  here, but is not a validated physical brake pressure/regen value.
- A second smaller no-pedal post-release recovery at `t=1995.61`
  peaked around +0.95 m/s²; other READY releases did not launch.
  A cruise setpoint above ego is **not sufficient alone** to explain
  the severe event.

## Factory camera CAN is *not* proven to reach the ECU

During the severe launch, the **camera-side bus-2** original 0x273
was `IS_ACCEL=1` while the **injected OP bus-0** 0x273 was DECEL or
neutral. Source-2 messages from the factory camera are present in the
rlog even when Panda is filtering them.

In this repository, `panda/board/safety/safety_toyota.h` installs a
`dnga_fwd_hook` that blocks camera `0x271` and `0x273` from bus
2 to bus 0. `panda/board/safety.h` associates it with
`SAFETY_TOYOTA`, selected by `selfdrive/car/dnga/interface.py`.
If that firmware and intended harness routing are actually deployed,
the apparent camera/OP command contradiction is *not* direct dual
actuation. The installed Panda mode, firmware, and harness forwarding
need independent verification on a bench. The source tree's safety
hook is highly permissive for transmitted whitelisted ACC IDs and
does not establish physical brake/accel safety limits.

## Controller evaluation

- R2 can sustain braking when lead/planner lookahead requests decel
  while downstream PID is positive. This disagreement is observed
  in long high-RPM events but a PID-positive veto could destroy
  legitimate closing-lead braking.
- `R2_RELEASE_FRAMES=80` itself does not explain a launch that starts
  *before its release phase ends*.
- Sending neutral `IS_ACCEL/IS_DECEL` while keeping `0x273`
  enabled and cruise `SET_SPEED` at 75 km/h may not remove all ECU
  torque authority. The hybrid ECU's actual acknowledgment and
  speed control state are unknown.
- Do **not** spoof the lead, remove the hybrid handoff veto, force
  `IS_ACCEL`, change the release timer, or command stronger braking
  without physical validation.

## Read-only tool and tests

`selfdrive/car/dnga/tests/v60_highway_handoff_audit.py` accepts an
aligned decoded CSV with time `t`, vehicle acceleration `a`,
`brake_state`, outgoing 0x273 modes, `accel` (downstream PID),
`p_a0`, RPM, driver pedal and optional decoded camera-side mode bits.
It reports four retrospective categories (neutral positive accel/
negative PID; neutral negative accel/positive PID; sustained high RPM
BRAKING/PID mismatch; and camera intent difference).
**It neither sends CAN nor runs on-road.**

```sh
python3 selfdrive/car/dnga/tests/test_v60_highway_handoff_audit.py -v
python3 selfdrive/car/dnga/tests/v60_highway_handoff_audit.py aligned.csv
```

The full CSV alignments, plots and analysis bundle were produced
from the user-uploaded logs and are not added to GitHub to avoid
publishing raw driving data.

## Engineering prerequisite to a live fix

Require HIL or professional automotive ECU instrumentation: confirm
**actual Panda firmware/safety mode and camera bus forwarding**;
ECU-side 0x271/0x273 acceptance; real factory ACC engagement state;
battery SOC and charge power; MG1/MG2 torque; engine fuel cut,
and brake pressure. A matched-speed stock-ACC baseline is not
available in the current logs. Re-evaluate single-controller
arbitration after those confirmations. **No live CAN/control changes
are included in this audit.**
