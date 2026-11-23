# V7.2 DNGA Panda safety development — bench-only, TX locked

**Status: OFFLINE SOURCE PROTOTYPE. NOT ROAD-QUALIFIED. DO NOT FLASH OR DRIVE THIS MODE.**

Base: `V7.1-R1`, keeping its original `dngaStock=28` mode,
`openpilotLongitudinalControl=False`, factory ACC and
steering-ratio `17.0` unchanged.

## Summary

The previous V7.1 Panda source audit demonstrated nine safety gaps in
model 28: unconditional `controls_allowed`, no monitored RX, accepting
malformed/unchecked steering messages and no driver-pedal override.

This V7.2 branch introduces a **separate safety model**:
- `SAFETY_DNGA_V72=29`, `cereal.CarParams.SafetyModel.dngaV72@29`.
- Header `panda/board/safety/safety_dnga_v72.h`.
- Original model `dngaStock=28` retained **unchanged** and still
  selected by `selfdrive/car/dnga/interface.py`.
- V7.2 model 29 starts with `controls_allowed=false`, never sets it
  true, and **its TX hook always returns 0 for every host command**.
  This includes OP steering 0x1D0 and longitudinal 0x271/0x273.
- Because it transmits nothing, model 29 forwards the OEM camera's
  **steering 0x1D0 as well** as 0x271/0x273/0x274 and other factory
  messages. It does NOT remove stock steering without an OP replacement.
- Bus0 -> bus2 and bus2 -> bus0 forwarding is unchanged; bus1 is not
  forwarded. The real Panda harness and relay have NOT been tested.

## Candidate checks (not used for actuation)

The new isolated C policy `dnga_v72_steer_candidate_ok` is
**only exercised by the native host-test harness**. It rejects:
- Wrong address, wrong bus, wrong DLC; bad 0x1D0 additive checksum.
- Missing observed 0x1D0 constant bits.
- Counter jumps/replay and request/torque inconsistency.
- Absolute magnitude greater than **255** and provisional rate changes
  greater than **20 up / 30 down** per 20-ms steering message.
- Disabled `controls_allowed`, unengaged OEM camera request, brake,
  accelerator, excessive measured driver torque.
- Missing or stale camera/driver/pedal feedback.

These 255/20/30 numbers are carried over from the current Python
controller as **unverified test envelopes**, NOT proven EPS limits.
Even if the candidate accepts a synthetic frame, actual
`dnga_v72_tx_hook` still returns 0 and blocks it.

## Preliminary RX observations (not validated ECU acknowledgment)

- 0x273 bus2: OEM additive checksum with a 50-ms expected period,
  guarded by a 150-ms freshness window. The camera engagement bit
  reports the **camera request only**.
- 0x0A1 bus0 brake: raw DBC BRAKE_ENGAGED bit 5.
- 0x277 bus1 gas: raw bytes 1-2, >100 heuristic currently used by
  V7.1 CarState.
- 0x0A4 bus0 steering module: DBC MAIN_TORQUE signed 12-bit field,
  provisional magnitude threshold 20.

These are hypotheses requiring actual raw CAN comparison. Brake/gas/
driver RX message checksums, timing, trustworthiness and ECU-state
correlations are **not** validated. Therefore no RX sequence,
including a valid engaged camera 0x273, can authorize TX in V7.2.

## Native host checks

Run from the repository root (developer computer or GitHub Actions):

```sh
cc -std=c11 -Wall -Wextra -Werror -O2 -I. \
  selfdrive/car/dnga/tests/native_v72_safety_test.c \
  -o /tmp/dnga_v72_safety_test
/tmp/dnga_v72_safety_test
python3 selfdrive/car/dnga/tests/test_v72_source_isolation.py -v
```

CI runs 35 native C cases and 80 Python cases across V7.2 isolation
and legacy V7.1/V7.0/V6 regression suites.

A **green CI result does not mean the actual Panda firmware or vehicle
is safe**. It demonstrates a fail-closed TX implementation and
synthetic-format checks in a host-native harness.

## STOP: hardware/vehicle validation still required

- Independently validate the real Yaris Cross HEV 0x1D0 actuator
  encoding, checksum, counter, EPS torque calibration and safe torque
  limits against full real OEM CAN traces.
- Independently verify driver torque, pedal states, brake, cruise
  engagement/ECU acknowledgment and received-message checksums.
- Compile the actual Black Panda firmware and exercise its forwarding,
  heartbeat/relay and fault paths in bench/HIL, including AEB and
  cancellation.
- Implement independently authenticated steering engagement and
  measured torque-rate safety before changing TX to anything other
  than unconditional DENY.
- Keep all OP longitudinal off while this is being developed. OP
  curve and brake shadows remain diagnostics only.
- Only a qualified vehicle safety validation process could justify a
  physical driving test after these gates.

**No full Panda firmware build, flash, or connected-vehicle validation
has been performed here.**

## Branch/source inspection (do not install)

```sh
cd /data/openpilot || exit 1
git fetch origin V7.2
git show FETCH_HEAD:panda/board/safety/safety_dnga_v72.h
git show FETCH_HEAD:selfdrive/car/dnga/tests/native_v72_safety_test.c
```

This only downloads and displays source. It does not switch the
running branch or change the device's Panda mode/firmware.

Keep your existing V7.1-R1 device software and Panda firmware
separate from this bench prototype. Note: the existing model 28 still
has documented serious steering safety deficiencies; retaining it is
not a certification of road safety.
