# V5.0 no-lead uphill curve: confirmed RPM rise and post-release acceleration

**Diagnostic only — no change to active longitudinal controller or Panda firmware.**
Source: user-provided `v5.0 curve brakle rev logs.zip`, recorded route
`2026-11-06--02-08-06`, segments 85–87.

## Evidence from native V5.0 rlogs

Decoded the original Cap'n Proto `carState`, `carControl`,
`longitudinalPlan`, `radarState`, bus-1 engine RPM `0x037`,
bus-1 pedal `0x277`, hybrid feedback `0x275`/`0x08C`,
OP source-0 `0x271`/`0x273`, and separate factory-camera
source-2 `0x271`/`0x273`.

The decisive no-lead event is **segment --87, relative 37.18–46 seconds**.
`longitudinalPlanSource=5` means **turn** in V5.0 cereal/log.capnp;
`hasLead=0` and the radar leadOne status are both false through the
release and surge. User accelerator CAN 0x277 remains zero.

| Window | V5.0 OP 0x271 | Engine RPM | Measured acceleration | OP 0x273 |
| --- | --- | --- | --- | --- |
| ~37.18–44.22 s | BRAKING 0x21 | ~1,900 rising to ~2,700–2,800 | speed falls from ~81.3 to ~75.9 km/h | IS_ACCEL=0, IS_DECEL=1, target ~vEgo |
| 44.274 s | READY 0x01 | ~2,711 | -0.25 m/s² | IS_ACCEL=0, IS_DECEL=0 |
| 44.888 s | READY 0x01 | ~3,046 from 20Hz alignment; **3,047 raw peak** | +0.96 m/s², controller PID -1.28 m/s² | IS_ACCEL=0, IS_DECEL=0, target ~vEgo |
| ~45.285 s | READY 0x01 | ~2,732 | **+1.293 m/s² peak**, controller PID ~-2.07 m/s² | IS_ACCEL=0, IS_DECEL=0 |

From 44.27–45.58 s, speed increases approximately **75.8 to
81.4 km/h** without an OP positive ACC mode or physical gas pedal input.
The planner remains mildly negative during the initial acceleration.
Engine RPM already rises during hydraulic braking and reaches its
maximum after the hydraulic command clears.

The 0x275 brake-request word is zero during this particular window,
while 0x08C friction rises during braking (~63 raw) and falls to zero
by ~44.7 s. This does not prove that regenerative or engine braking is
absent; hybrid raw torque semantics are not Techstream-validated.
Rpm 0xffff/0xfffe samples represent invalid/sentinel measurements and
are excluded from peak calculation.

## Competing factory ACC request

At ~44.25–45.60 s:
- Factory camera bus-2 0x273: **27/27 IS_ACCEL=1**, IS_DECEL=0,
  desired speed roughly 74.86–78.14 km/h.
- OP source-0 0x273: **27/27 IS_ACCEL=0 and IS_DECEL=0**, desired
  speed ~current vEgo.
- Factory-camera 0x271 is READY; OP 0x271 is also READY.

The V5.0 source file `panda/board/safety/safety_toyota.h`
`dnga_fwd_hook()` **blocks bus-2 0x271 and 0x273 forwarding to bus 0**
when that hook is running. The V5.0 DNGA interface selects Toyota
safety model 2, whose hooks map to `dnga_hooks` in `panda/board/safety.h`.
**This is source-code evidence only, not proof of the actual flashed Panda
firmware or that a physical relay/harness had its expected state.**
No inference that the camera command reached the ECU is warranted.

## Interpretation and next steps

This is distinct from the Nov 5 **lead-following** brake-release chattering:
it is a no-lead, planner-source=turn hydraulic brake event, followed by
unexpected measured acceleration while OP asks for negative acceleration
and sends neutral 0x273. A more aggressive brake command, deeper
negative 0x273 target or blanket coast-release timer is **not justified**.
A plausible explanation is HEV/ECU drivetrain behavior after 0x271
brake-release, but it is not confirmed. The raw engine RPM rise also
starts before 0x271 release.

R2's off-car probe now logs `curve_brake_active`, `source`,
`turn_speed`, `curve_speed_error`, measured `engineRPM`, and
pedal overrides in the clear/entry context. Re-run the exact-controller
replay on this Nov 6 archive in a matching NEOS/0.8.13 environment.
Compare with the OEM camera bus-2 0x273 and verify the actual Panda
safety/harness configuration before considering any actuation patch.

**Safety:** do not deploy a revised OP longitudinal controller on
public roads until this uncommanded acceleration is investigated and
independently validated. Driver remains responsible for overriding
unexpected vehicle response.

## Nov 5 replay ordering caveat

The uploaded Nov 5 replay reported 46,679 equal V5.0/probe commands,
zero differences and 99.6887% brake-state agreement. However, the
replay.log shows annotated `--14-questionable behavior` and
`--15-questionable behavior` folders were processed BEFORE `--0`
because the previous sorter did not recognize numeric prefixes with
suffix text. A corrected R2 sorter and regression test now preserve
chronological order; **repeat the Nov 5 baseline replay** before using
internal events from the affected route.
