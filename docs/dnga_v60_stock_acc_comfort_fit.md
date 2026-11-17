# DNGA v6.0: stock-ACC-derived brake / ACC_CMD comfort fit

**Status: OFFLINE experimental calibration.** The live R2 selector in
`carcontroller.py` stays `V6_R2_ACTIVE_CAN = False`.
This is not evidence of ECU safety or physical stopping performance.

## Source records

- November 8, 2026 archive: `2026-11-08--21-24-53--0` through
  `--7`; factory ACC enabled in segments `--2` through `--6`.
- Four older uploaded factory ACC rlogs `127`, `128`, `129`, `130`.
- Decoded factory camera CAN frames on bus 2: `0x271` (state,
  pump, magnitude), `0x273` (enable, speed target, mode bits), and
  same-stream `carState.vEgo`.
- Accepted checksum-valid `0x271`/`0x273` pairs within 60 ms,
  with the closest measured `vEgo`; used 20 Hz BRAKING frames.

Observed **28 BRAKING episodes** and **1,142 paired BRAKING frames**.
For the fitted target-gap model, retained **1,047** normal-domain
frames with 8–80 km/h, decoded 0x271 strength 0.08–0.87, and
observed 0x273 gap between -1 and 12 km/h. This omits the more
aggressive stock 0.271 pump-6/7/8/9 requests; v6.0 does NOT model
that stronger OEM domain.

## Stock 0x271 magnitude

For moving BRAKING frames with 0x271 magnitude integer `M`:

```
stock_strength = (200 - (M & 0xff)) / 100
```

The upper byte is the pump level and is **not** a directly measured
hydraulic brake torque. The median *first-frame* stock strength across
28 episodes was 0.305 (range 0.24–0.49).
Among positive adjacent strength changes within continuing stock
BRAKING episodes, the median increment was +0.020 per ~50 ms.

R2 retains its previously specified **first ordinary braking**
strength 0.080, encoded with the original V5.0 encoder as `1216`.
Later ordinary moving-brake rise is bounded by **0.75 × OEM median
positive step = +0.015 per 50 ms**. That maximum is further limited
by the inherited V5.0 brake target filter, speed/curve/lead limits
and measured-deceleration governor. Urgent/emergency and low-speed
stop/hold do not use the comfort-only slew override.

These observations DO NOT identify the hidden OEM planner input or
a physical deceleration-versus-magnitude transfer function. R2 still
uses its own planner demand and inherited V5.0 brake caps.

## Stock 0x273 ACC_CMD target scaling

For the same 1,047 normal-domain frames, fit the OEM-requested speed
gap (vehicle speed minus ACC_CMD target), with speed `V` in km/h and
encoded strength `B`, via robust least squares:

```
stock_gap_kph = -0.31592604 + 7.01016383 * B + 0.03393986 * V
```

Fitting details: SciPy soft-L1 loss, `f_scale=0.5`, with model
inputs clamped to the observed speed and strength domain at runtime.

In-sample mean absolute error: **0.221 km/h**; RMSE: **0.295 km/h**;
95th-percentile absolute residual: **0.607 km/h**.
These are command-fitting errors, NOT vehicle-acceleration or
stopping-distance errors. Do not extrapolate OEM validity beyond
this observed range.

For normal R2 moving BRAKING, set the offset target to
`0.85 * stock_gap_kph` and retain the existing
**+0.18/-0.35 km/h per 50 ms** target slewing and the
current maximum gap. For urgent cases, do not apply the
0.85 comfort attenuation. The command remains `IS_DECEL=1`,
`IS_ACCEL=0` on R2 BRAKING.

Neither the 0x273 fit nor the 0x271 median is a validated motor,
regenerative torque, hydraulic pressure, or tire force model.
An unexplained earlier positive-acceleration-on-release event remains
unresolved.

## Tests and non-claims

The CI test suite runs real V5.0 encoder/filter/cap source functions
in isolated fixtures. It checks initial `1216`, normal +0.015
slew when demand requires it, V5.0 speed caps, original urgent
slew, 85% normal and 100% urgent fitted 0x273 targets, target
slewing, and inherited SNG method selection.

These tests are NOT native onroad processing, hardware Panda
transmission, brake pedal override, OEM ECU response, brake force,
emergency stopping distance, or the actual R2-to-SNG boundary
under a moving vehicle. Those validation gates remain open.
