# DP 0.8.13 + DNGA V2.5S + LegacyPilot 0.8.16 Hybrid Model

This package keeps the `nightly` branch's DNGA Yaris Cross vehicle code and
V2.5S acceleration/braking safeguards. It changes only the road-camera driving
model:

- Default: `jefranc15/legacypilot` 0.8.16 hybrid `supercombo`
- Fallback: original DragonPilot 0.8.13 `modeld`
- Driver monitoring: unchanged DragonPilot 0.8.13 model
- DNGA steering, acceleration, regen, hydraulic braking, DBC, and panda safety:
  unchanged from `nightly`
- Black Panda firmware, `pandad`, `boardd`, and flashing behavior: unchanged

The installer builds the LegacyPilot source on the comma two against
DragonPilot 0.8.13's cereal and VisionIPC libraries. It does not install R2's
prebuilt camera, board, Panda, or model-runner binaries. Those binaries use a
newer runtime/IPC stack and were the likely source of the compatibility problems
seen when trying R2 with a Black Panda.

This package intentionally does not repair an already unrecognized Black Panda.
It isolates the model experiment so the currently working `nightly` Panda stack
is not altered or reflashed.

## Source pins

- DragonPilot/DNGA base:
  `jefranc15/dragonpilot@63a7a7fce712d19e05f5062abcd02135423864f7`
- Hybrid runner source:
  `jefranc15/legacypilot@e355ea553a17a0a64ef5534239233e4c9e5ad198`
- Precompiled 0.8.16 model data:
  `jefranc15/dragonpilot:r2@c1024d1d000047c49419b30145a338c069a343d7`
  (`supercombo.thneed` only)
- `supercombo.thneed` SHA-256:
  `17431530e6134dfd40bf35bfa4d764e2789a93f81561ac17cd938ec84e41f35f`

## Selecting the model

The hybrid model is selected by default. To return to the original 0.8.13
driving model:

```sh
touch /data/media/0/use_dp_0813_model
reboot
```

To re-enable the 0.8.16 hybrid model:

```sh
rm -f /data/media/0/use_dp_0813_model
reboot
```

## First test

1. Apply and compile while parked, powered, and cooled.
2. Confirm there is no `modeld` crash or camera IPC assertion.
3. Check model execution time, dropped frames, and device temperature.
4. First drive with openpilot lateral control and stock longitudinal.
5. Enable experimental DNGA openpilot longitudinal only after confirming stable
   lead detection and reviewing logs.

The 0.8.16 model has internal stop-line outputs, but the retained 0.8.13
planner does not use them. This package does not add traffic-light, stop-sign,
or autonomous intersection behavior.
