# V7.0-stocklong — factory longitudinal + read-only brake-smoothing shadow

**STATUS: ENGINEERING / BENCH-ONLY PROTOTYPE. NOT ROAD-VALIDATED.**

Base: `v6.0` commit `aa94bc407328dfa36c553145cd48fc7619fcfe03`.
Motivation: Nov 17 highway `0x271`/ `0x273` command-response
mismatch, sustained revving, and recorded no-pedal +1.89 m/s²
acceleration following brake release. No new OP brake actuation is enabled.

## Architecture

- The factory camera is the sole longitudinal command owner.
- The controller transmits only `0x1D0` (OP steering). It does not send
  `0x271` (brake), `0x273` (ACC speed/mode), `0x274` (stock HUD), AEB
  spoofing, ACC button messages, or a synthetic lead flag.
- The dedicated Panda model `dngaStock=28` forwards *original camera*
  `0x271`, `0x273` and `0x274` verbatim; blocks the camera
  `0x1D0` so OP owns steering. Its TX filter permits only `0x1D0`
  on bus 0. This is not the legacy `SAFETY_TOYOTA=2` V6 hook.
- The camera messages are decoded in `interface.py` for telemetry.
  `carstate.py` shows camera intent only while fresh, and suppresses a
  no-lead below-30 km/h *engaged* indication, but **there is no verified
  ECU actuation acknowledgment**. This is a remaining safety limitation.
- `stock_long_shadow.py` proposes a hypothetical low-demand brake ramp
  for later analysis, but its result never enters `can_sends` or any
  actuator path. Strong-brake and release requests are represented as
  unchanged in the shadow evaluation. 1Hz diagnostic lines may appear
  as `V7_STOCKLONG_SHADOW`.
- Curve braking is **not active**. OP steering continues; factory ACC
  owns speed and all braking. Above/below 30 km/h the factory's native
  capabilities and limitations apply.

## Critical compatibility warning

An old V6 Panda with safety model 2 is **not compatible** with the new
V7 model 28. Switching only Python/Git code without installing matching,
bench-tested firmware can disable or disrupt ACC and steering. A Panda
status entry that merely says `safetyModel=28` does **not** independently
prove correct forwarding or calibrated brake behavior.

The V7 safety code has NOT been road- or HIL-validated. In particular,
its inherited steering gating uses a software latch rather than
production-qualified Panda RX checks/driver-override protections.
Do not use this branch to operate the vehicle on public roads.
Do not attempt to enable the shadow output as brake commands.

## Parked-device checkout / read-only checks

Run with the car **parked, ignition off and not driving**. This checks
out the source but does *not* validate or install matching Panda firmware:

```sh
cd /data/openpilot || exit 1
git status --short
git fetch origin V7.0-stocklong || exit 1
git switch -c V7.0-stocklong --track origin/V7.0-stocklong \
  || git switch V7.0-stocklong || exit 1
git rev-parse --short HEAD
grep -n 'openpilotLongitudinalControl = False\|SafetyModel.dngaStock' selfdrive/car/dnga/interface.py
grep -n 'SAFETY_DNGA_STOCK' panda/board/safety.h
python3 -m py_compile selfdrive/car/dnga/carcontroller.py selfdrive/car/dnga/stock_long_shadow.py
python3 selfdrive/car/dnga/tests/test_v70_stocklong_shadow.py -v
```

**Stop here.** Do not reboot into on-road operation or connect this
branch to an active vehicle based on passing offline tests alone.
Panda boot/firmware flash and physical CAN bridging verification must
be coordinated together under professional bench/HIL supervision.

### Return to previous source branch (parked)

```sh
cd /data/openpilot || exit 1
git fetch origin v6.0 || exit 1
git switch v6.0 || git switch --track origin/v6.0 || exit 1
git rev-parse --short HEAD
```

This reverts the **source checkout only**. If Panda firmware has been
flashed independently, its compatibility must also be restored before
vehicle operation. The prior V6 longitudinal implementation itself
is associated with the reported hard-launch issue: switching back is
not an endorsement of V6 highway longitudinal operation.

## Bench acceptance criteria before any active deployment

1. Verify current Panda firmware hash/safety model 28, RX/TX/forwarding
   using an independent bus-side CAN recorder, and no duplicate camera
   or OP ACC actuation.
2. Validate preserved OEM `0x271`, `0x273`, `0x274` checksums,
   counters, timing, AEB priority, cancellation, driver brake override,
   and factory minimum-speed behavior; establish a real ECU
   longitudinal-state/acceptance signal.
3. Test camera-loss, Panda reset, battery changes and engine braking
   transitions on the bench. Verify that OP steering is inactive when
   assistance cannot be reliably confirmed, and implement proper
   Panda lateral torque/driver-override limits.
4. Only after OEM passthrough behavior is independently established,
   assess shadow proposed-vs-OEM braking on representative stock ACC
   logs and prove no required braking is reduced before considering
   any active smoothing.
5. Separately research OEM-supported curve setpoint management. No
   OP curve braking is implemented on V7.

### Offline CI
`python3 selfdrive/car/dnga/tests/test_v70_stocklong_shadow.py -v`
checks brake filter state, camera freshness, exclusive OP CAN ownership
and the unique Panda safety model contract. Python compilation and
static source checks are NOT substitutes for Cap'n Proto/Panda firmware
compilation and physical testing.
