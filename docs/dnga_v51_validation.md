# DNGA V5.1 development validation (2026-11-05 recordings)

Base: `V5.0` commit `ce841812a5b17a6bdcac9b4e9deb5235f06978b9`.
Vehicle: Toyota Yaris Cross HEV AC200, OnePlus 3T, DragonPilot 0.8.13.

**Status: development / OFF-ROAD ONLY.** This branch is not validated for public-road autonomous longitudinal use. The 2026-11-05 archive exhibits revving with commanded braking and weak measured deceleration; no actuator-strength or Panda safety change is justified yet.

## Lateral change in this branch

V5.0 called `LanePlanner.get_d_path(v_ego, t_idxs, self.path_xyz)`. Since `get_d_path` changes its input ndarray in place, subsequent E2E selection returned the **lane-blended** path rather than the original model trajectory.

V5.1:
- Builds independent model/E2E and lane-blended candidates from separate ndarray copies.
- Translates modelV2's camera-relative path by `LanePlanner.camera_offset` exactly once in both modes. Lane lines have already been camera-translated by `LanePlanner.parse_model`.
- Applies `LanePlanner.path_offset` once to either path.
- Preserves the original model path for heading interpolation.
- Leaves lateral PID/INDI, steer ratio (18.35), `static_paramsd`, CAN, Panda, and longitudinal control untouched.

In this legacy UI convention, `dpCameraOffset=+4` means internal `camera_offset=-0.04 m`; `dpPathOffset=+5` means `path_offset=-0.05 m`. These are **configuration sign conventions**, not independent proof of the lens mounting direction.

Run focused offline unit tests with `PYTHONPATH=. python -m unittest selfdrive.controls.lib.tests.test_dnga_path_utils -v`.

## Factory-camera / DragonPilot rlog comparison

Rlogs contain distinct raw inputs:

- `can` RX source `2`: factory camera `0x271` / `0x273` message.
- `sendcan` TX source `0`: DragonPilot `0x271` / `0x273` message.
- `can` RX source `128`: echoes of DragonPilot's transmissions; **not** independent OEM requests.
- `0x275`, `0x08C`, `0x2C9` are on a different bus and should be interpreted separately.

Factory `0x271` was observed in BRAKING (state `0x21`) before DragonPilot in the following actual approaches (seconds into segments):

| Segment | OEM BRAKING starts | OP BRAKING starts | OP minus OEM |
| --- | ---: | ---: | ---: |
| `14-45-36--6` second approach | 19.700 s | 22.007 s | +2.307 s |
| `14-45-36--6` third approach | 53.300 s | 54.058 s | +0.758 s |
| `14-45-36--14` first approach | 15.400 s | 18.425 s | +3.025 s |

These are same-journey CAN command timing differences, **not** measured actuator latency nor proof of stock ACC safety/strength under the same mode.

Other findings:

- `14-45-36--6` at 0.407–2.106 s: OP requested braking while the factory camera stayed READY; the driver then used the foot brake (~2.16 s) after weak initial deceleration. That event cannot be classified as OP failing to request brakes.
- `14-45-36--14` at 44.376–46.176 s: OP BRAKING while camera remained READY; RPM rose ~1371 to 2800 and friction feedback increased while vehicle speed stayed near 40 km/h. OP `IS_ACCEL=0`. At ~46.195 s the driver used CANCEL.
- `11-32-49--7`: manual brake at ~101.8 km/h; neither factory nor OP had issued `0x271 BRAKING` in the segment. Radar lead had appeared ~130 m ahead shortly before intervention; cause remains inconclusive.
- Across prior full-archive OP analysis, five short brake/coast/brake reentries were observed; one ~0.15 s gap merits checking for an urgent or stop-guard bypass.

Raw factory camera data **is present** in these logs. Its presence does not imply that stock ACC was engaged or that the OEM command actually actuated the vehicle while OP was in control. In the current code `CarState.stock_acc_cmd` / `stock_brake_mag` are initialized but not populated from a dedicated factory-camera parser, so live control must not simply read those as verified source-2 signals.

## Next acceptance gates (longitudinal)

1. Synchronize OEM RX, OP TX, radar lead, planner, PID, driver input, `0x275` brake feedback, `0x08C` friction and `0x037` RPM. Compare transitions at 20 Hz without look-ahead alignment.
2. Separate planner-late versus controller-entry-late versus physical-response-late. Replay all named episodes and additional matched approaches without changing actuation.
3. Confirm raw `0x271` state/pump/magnitude meanings before modifying any brake request, and verify safety boundaries in independent tests.
4. Preserve the `0x275`/`0x08C` acceleration veto, logging-only hybrid torque fields, existing coast-first neutral `0x273` policy, and SNG HOLD/CREEPING.
5. Do not enable longitudinal actuation changes for road use until closed-course tests establish appropriate actual braking response, braking/rev mitigation, no unexpected surges, and safe urgent re-entry.

## Next acceptance gates (lateral)

1. Use modelV2 versus `lateralPlan.dPathPoints` and `dPathWLinesY` in offline replay: the E2E selected path must now be independent of lane blending.
2. Confirm physical camera lens position and lateral sign. Preserve the intentionally fixed 18.35 steer ratio/zero angle-offset source until separately justified.
3. Verify path offset at +/-5 cm in synthetic and safe controlled recordings, then compare desired curvature, steering angle/torque and actual lane geometry.
4. Do not assume path separation fixes right bias automatically: logged raw E2E path was frequently about 18–22 cm to the right of the lane-blended path in high-confidence subsets. This may expose a different, potentially greater bias.

Branch is for review, test, and controlled validation—not for public-road deployment.
