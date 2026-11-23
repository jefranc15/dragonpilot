# DNGA V7.1-R1 Panda validation — native source audit

**Overall verdict: FAIL (NOT ROAD-QUALIFIED).** This branch is an **offline audit only**;
it must NOT be deployed as a new Panda firmware or interpreted as a successful
steering safety qualification.

Base: `V7.1-R1`. Panda safety source was **not changed** by this audit.

## What was actually tested

- Compile a host-native C harness against the **unchanged original**
  `panda/board/safety/safety_dnga_stock.h`.
- Invoke its `dnga_stock_hooks.init/rx/tx/fwd/tx_lin` with controlled frames.
- Cross-check the board's safety registry, `cereal/car.capnp`,
  `selfdrive/car/dnga/interface.py`, `boardd.cc`, and common
  `panda/board/safety.h` guards.
- Re-run V7.1-R1, V7.1 and V7.0 Python regression tests to confirm
  this audit branch has not modified the controller.
- **No vehicle, CAN transceiver, firmware image, Black Panda USB session,
  EPS ECU or real forwarding path was exercised.**
- Test's CAN packet structure and GET macros are host-side stand-ins
  matching the original hook's minimal interfaces. These tests evaluate
  decisions made by the real source hook, not the entire Panda firmware.

## Native-C results (23 cases)

### 14 intended policy checks: PASS

| Check | Result |
|---|---|
| Initialize stock safety hook | PASS |
| Host steering `0x1D0` bus0 accepted | PASS |
| `0x1D0` host TX on bus2 rejected | PASS |
| OP brake `0x271` TX rejected | PASS |
| OP ACC `0x273` TX rejected | PASS |
| OP HUD `0x274` TX rejected | PASS |
| Unlisted OP `0x275` TX rejected | PASS |
| Factory camera `0x271` bus2 to bus0 forwarded | PASS |
| Factory camera `0x273` bus2 to bus0 forwarded | PASS |
| Factory camera `0x274` bus2 to bus0 forwarded | PASS |
| Camera `0x1D0` bus2 to bus0 blocked | PASS |
| Powertrain bus0 to camera bus2 forwarded | PASS |
| Non-forwarding bus1 handled as `-1` | PASS |
| LIN TX rejected | PASS |

### 9 safety gaps reproduced — release blockers

| Confirmed unsafe behavior | Severity / why |
|---|---|
| `controls_allowed=true` during initialization | Critical — actuation permission not linked to engagement |
| No monitored RX safety messages (`dnga_rx_checks.len=0`) | Critical — no validated/fresh vehicle state |
| Steering `0x1D0` accepted with invalid DLC | High — malformed host frames not blocked |
| Steering `0x1D0` accepted with invalid checksum | High — corrupt host frames not rejected |
| Arbitrary `0x1D0` payload accepted | Critical — steering command magnitude and metadata unchecked |
| Steering TX accepted with `controls_allowed=false` | Critical — explicit disengagement does not gate TX |
| Steering TX accepted with gas and brake override | Critical — no driver override gating |
| Arbitrary unknown RX packet accepted | High — no RX whitelist/integrity checks |
| RX does not clear `controls_allowed` under override | Critical — no vehicle-specific disengagement enforcement |

**Do not confuse a green CI run with a safe safety hook.**
The audit's native runner intentionally returns success *when it proves
the known unsafe behavior*, so the evaluation can be reproduced.

### Existing framework does not close these gaps

`panda/board/safety.h` guards host TX and forwarding on
`relay_malfunction` and contains common rate-check helper functions.
The DNGA stock hook does not invoke its steering checks. The older
`safety_toyota.h` supplies `dnga_rx_checks` with `len=0`.

`panda/board/main.c` contains heartbeat-loss fallback to SILENT.
That is a coarse USB connectivity policy, not a substitute for
per-frame steering limits, driver override or ACC disengagement.

`boardd.cc` selects the Panda safety model from the decoded
`CarParams.safetyConfigs` when it is ready. The source agrees on
model ID `dngaStock=28`, safety param 1, but an actual Black Panda
may have different firmware. The script does not verify its flash state.

## Tests

```sh
cc -std=c11 -Wall -Wextra -Werror -O2 -I. \
  selfdrive/car/dnga/tests/native_v71_panda_audit.c \
  -o /tmp/native_v71_panda_audit
/tmp/native_v71_panda_audit
python3 selfdrive/car/dnga/tests/test_v71_panda_source_audit.py -v
```

CI adds the already-passing 15+27+15 V7-series Python checks.
The original V7.1-R1 74-test workflow was completed separately.

## Remaining real-world qualification gates

1. **Safety code fixes, not just tests:** develop a separate safety
   implementation with independently enforced torque magnitude, per-frame
   and real-time rate limits, neutral request when not engaged,
   driver override, checked RX integrity/freshness and fault handling.
   Use DNGA/Yaris Cross-specific measured limits; do not blindly take
   BukaPilot's limits or the Python controller's 255 saturation.
2. Validate `0x1D0` checksum, counter, signal encoding and stock
   feedback with recorded CAN data.
3. Bench-test relay state, fault injection, mismatched Panda firmware,
   missing CAN frame timing, driver override, and exact stop of
   steering TX on disengagement.
4. Bench-test stock camera `0x271`/`0x273`/`0x274` passthrough on
   actual Panda hardware. Confirm AEB and failure behavior.
5. Confirm actual Black Panda hardware type, firmware version,
   `pandaStates.safetyModel=28`, Panda health, observed camera/ECU
   traffic and vehicle response using *non-driving instrumentation*.
6. Only after independent safety validation, consider
   closed-track testing by qualified personnel.

The present branch is for diagnostics and code review only. No new
firmware was built/flashed, and no Panda safety behavior was altered.
