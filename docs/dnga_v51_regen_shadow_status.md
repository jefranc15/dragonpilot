# v5.1-Regen — R2 V4 stock-coordinated braking shadow

**Current integration status (2026-11-09): RESEARCH / DIAGNOSTIC ONLY.**
This branch is **not a road-ready regenerative-braking controller**. It does
not replace the V5.0 brake or acceleration CAN commands.

## Integrated source

- `selfdrive/car/dnga/r2_v4_shadow_controller.py`: new self-contained 20 Hz
  counterfactual state machine that computes what R2 V4 *would* send.
- `selfdrive/car/dnga/carcontroller.py`: keeps its V5.0 longitudinal producer,
  the existing final interlock and both `dnga_create_accel_command` /
  `dnga_create_brake_command` unchanged. A read-only observer is invoked
  **after** the V5.0 CAN transmit list is constructed, solely to record
  comparisons via `R2V4_SHADOW_ONLY` cloudlog messages.
- `selfdrive/car/dnga/tests/test_r2_v4_shadow_controller.py`: eleven
  focused regression checks for simulated weak/strong braking, target slew,
  source bridging, release, feedback and out-of-domain diagnostics.
- Panda safety mode, firmware, and forwarding hooks are unchanged.

## Candidate logic — not physical authority

- Mild active BRAKING starts at `0x271 MAGNITUDE=1216`, pump `-0.4`.
  Stronger candidate requests scale progressively with the old V5.0 encoder;
  the simulated high-demand pump step is `-0.5`, not a certified maximum.
- `0x273 ACC_CMD` follows a fitted *factory command* offset and is shaped
  with a maximum ordinary gap increase of 0.18 km/h per 20 Hz tick.
  A separate urgent 0.60 km/h step is **not validated**.
- Following a `turn` or lead-source change to `cruise`, continue hypothetical
  braking only when both current planner and downstream PID request deceleration
  and measured deceleration is not already excessive. The chosen planner
  threshold is `-0.14 m/s²`; this is a **replay hypothesis**, not a calibration.
- A normal BRAKING->READY transition models `1224 / -0.4` for a bounded
  release period, then ordinary `200 / 0`. This is not a proven RPM cure.
- Simulated ACCEL requires distinct fresh brake-clear `0x275` and
  `0x08C` feedback and compatible planner/PID values. This gate can only
  veto an OpenPilot ACCEL request: it cannot guarantee that the hybrid ECU
  will not independently command positive wheel torque.
- The observer flags absent negative `0x275` during hypothetical BRAKING,
  but it cannot know the counterfactual ECU response.

## November 8 stock/manual findings

- 988 genuine factory BRAKING samples across 27 entries; every observed
  active-braking `0x275` first word was negative in that session.
- Stock `0x271` low-byte change and `0x275` first word had correlation
  about 0.9994. That relationship is consistent with an *ACC coordination
  request*, **not** a calibrated MG2 torque or HV-battery current measurement.
- 949 valid new stock frames cross-checked the target-field fit: MAE
  ~0.20 km/h. Most samples are below 79 km/h, so this does not validate
  the motor's response above 90–100 km/h.
- The D/B `0x348` and `0x17A` raw signals remain **undecoded**.
  Their apparent correlation with braking is not evidence of amperes or Nm.
- Seven stock BRAKING frames used a -0.6 pump: outside the candidate's
  previously investigated pump branch.

## Validation scope and blockers

Eleven focused standalone V4 module unit tests passed in a separate Python
environment. Prior full V4 replay suite had 38 V4 tests and 48 unchanged
V3 regression tests, **but those were for the previous standalone offline
module, not this live-embedded observer adapter**. No device-level import,
CAN packer regression, HIL, or physical brake/regen measurement is claimed.

This build intentionally leaves R2 V4 **non-actuating** because:
1. 251 prior counterfactual BRAKING frames disagreed with V5.0 READY;
2. The original V5.0 run suffered an unexpected acceleration surge even with
   `IS_ACCEL=0`;
3. The hybrid ECU response to new `0x271` and `0x273` combinations is unknown;
4. Urgent and emergency deceleration authority is not demonstrated; and
5. B-mode and manual signals do not yet establish HV battery power or MG2
   wheel torque.

**Do not enable the counterfactual outputs as actual CAN commands based only
on these tests.** Retain V5.0 for passive factory ACC / D / B logging, with
OpenPilot disabled (dashcam mode) and normal OEM operation confirmed.
