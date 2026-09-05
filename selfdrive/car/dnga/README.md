# DNGA Yaris Cross HEV

This port retains the behavior and tuning of `nightly` at
`12c70379a1661c4783df10fe5730a45258567a72` while separating the controller's responsibilities.

- `carcontroller.py`: steering limits, CAN scheduling, HUD, and actuator feedback.
- `longitudinal.py`: lead qualification, hydraulic braking, stop-and-go, brake release,
  hybrid feedback supervision, and desired-speed shaping.
- `values.py`: vehicle identification, steering parameters, longitudinal tuning, and brake states.
- `carstate.py`: vehicle state, button handling, and ACC/LKAS latches.
- `interface.py`: car parameters and validated passive CAN observation.
- `dngacan.py`: outgoing CAN encoding and checksums.
- `dnga_hybrid_feedback.py`: read-only hybrid/brake decoding and freshness checks.

The cleanup removes revision prefixes, unused tuning and helpers, write-only diagnostic
state, a redundant brake-mode variable, and unreachable curve/negative-target regen paths.
Engagement resets share one method. Per-cycle plan, lead, brake, session, and propulsion
decisions have named fields instead of long positional argument lists. These snapshots
retain the original ordering when latches change during a control cycle.

Active confirmation counts and deadlines remain in place because they affect behavior.
`*_COUNT` parameters count 20 Hz longitudinal updates; `*_FRAMES` parameters use the
100 Hz car loop. Steering remains at 50 Hz and longitudinal/HUD messages at 20 Hz.
Controller gains, actuator limits, CAN layouts, DBCs, and Panda safety policy are unchanged.

## Offline comparison

From a checkout containing the baseline commit, with `cantools` installed:

```sh
python selfdrive/car/dnga/tests/compare_behavior.py
```

The standalone harness loads the original and refactored implementations in one isolated
process. It simulates messaging and vehicle inputs, executes both CAN encoders, and compares
unquantized encoder inputs, DBC-packed bytes, actuator feedback, and fault outputs.
It is intentionally separate from automatic test discovery because it replaces hardware
and messaging modules inside its process.

The checks cover engagement and pedal overrides, all three distance settings, lead braking
and loss, stock and predictive stop guards, crawl/hold/resume, highway handoff, low-speed
wake and overshoot, stale/malformed subscribers, hybrid freshness/agreement/overlap faults,
and explicit rearm. Separate checks compare 6,000 button cycles, all 65,536 raw RPM values,
hybrid and stock-frame decoders, car parameters, passive observation, and `opendbc/dnga_hev.dbc`.

Validation: 11 checks passed, including 64,623 control frames and 107,710 emitted CAN messages.
All five brake states and all four longitudinal fault reasons were exercised. Python 3.8
syntax and static checks passed. The comparison uses simulated I/O and a DBC packer;
it does not replace native-device, Panda, or vehicle testing.
