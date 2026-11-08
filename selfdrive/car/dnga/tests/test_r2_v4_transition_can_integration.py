"""R2 V4 end-to-end offline CAN integration and transition audit.

Uses the REAL CarController.update(), real DNGA command packer, and real V4
shadow and actuator. Replaces ONLY the external V5.0 longitudinal planner
with a deterministic recorded-state fixture; it does not model Toyota ECU
dynamics, braking force, timing of controlsd disengagement, or Panda hardware.

REQUIRES the compiled device opendbc/can/packer_pyx and cereal/messaging libs.
Never opens a Panda connection; no CAN messages are transmitted.
"""
import unittest
from dataclasses import replace
from types import SimpleNamespace

from opendbc.can.packer import CANPacker

from selfdrive.car.dnga.carcontroller import CarController
from selfdrive.car.dnga.dngacan import dnga_checksum
from selfdrive.car.dnga.longitudinal import LongitudinalCommand
from selfdrive.car.dnga.r2_v4_experimental_actuator import R2V4ExperimentalActuator
from selfdrive.car.dnga.r2_v4_shadow_controller import R2V4ShadowController
from selfdrive.car.dnga.values import BrakeState, CAR, DBC


class FakeActuators:
  def __init__(self, accel=0.0):
    self.steer = 0.0
    self.accel = float(accel)

  def copy(self):
    return FakeActuators(self.accel)


class SyntheticV50Source:
  """V5.0 command fixture; real CAN packing and CarController run downstream."""
  def __init__(self):
    self.plan_frame = 0
    self.plan_source = "turn"
    self.plan_accel = -0.2
    self.plan_accel_next = -0.2
    self.plan_curve_accel = -0.2
    self.plan_lead_accel = -0.2
    self.plan_has_lead = False
    self.block_brake_until_frame = 0
    self.urgent_brake = False
    self.handoff_pending = False
    self.handoff_ready_counter = 0
    self.kinematic_ready_counter = 0
    self.speed_offset = 0.0
    self.command = LongitudinalCommand(
      True, False, 20.0, True, False, BrakeState.READY, 0.0, 200
    )

  def update(self, enabled, CS, frame, accel, pcm_cancel_cmd, lead):
    return replace(self.command)


class TestActualCarControllerTransition(unittest.TestCase):
  def setUp(self):
    self.cs = SimpleNamespace()
    self.cs.out = SimpleNamespace(
      vEgo=20.0, aEgo=-0.2, steeringTorqueEps=0.0,
      leftBlinker=False, rightBlinker=False, steeringPressed=False,
      brakePressed=False, gasPressed=False,
      cruiseState=SimpleNamespace(available=True, enabled=True, speed=20.0),
    )
    self.cs.lkas_latch = False
    self.cs.cruise_speed = 20.0
    self.cs.op_distance_val = 2
    for name in (
      "stock_fcw", "stock_aeb", "stock_adas_frontDepartureHUD",
      "stock_lkc_off", "stock_fcw_off",
    ):
      setattr(self.cs, name, False)
    for suffix in ("275", "08c", "2c9", "12a", "125"):
      setattr(self.cs, "hybrid_feedback_rx_frame_" + suffix, -1000000)
    self.cs.hybrid_brake_request_raw_275 = 0
    self.cs.hybrid_friction_raw_08c = 0
    self.lon = SyntheticV50Source()
    self.car = CarController.__new__(CarController)
    self.car.last_steer = 0
    self.car.steer_rate_limited = False
    self.car.params = SimpleNamespace(
      STEER_BP=[0.0, 30.0], STEER_LIM_TORQ=[255.0, 255.0],
      STEER_DELTA_UP=22, STEER_DELTA_DOWN=35,
    )
    self.car.packer = CANPacker(DBC[CAR.YARISCROSSHEV]["pt"])
    self.car.need_clear_engine = False
    self.car.stock_ldw = False
    self.car.longitudinal = self.lon
    self.car.r2_v4_shadow = R2V4ShadowController()
    self.car.r2_v4_actuator = R2V4ExperimentalActuator()
    self.car.r2_v4_last = None
    self.car.r2_v4_shadow_errors = 0

  def tick(self, frame, plan=-0.2, pid=-0.2, feedback=0, friction=0,
           base_state=BrakeState.READY, base_accel=True, enabled=True,
           brake_pressed=False, gas_pressed=False, source="turn",
           feedback_fresh=True, measured_accel=-0.2):
    assert frame % 5 == 0
    self.lon.plan_frame = frame
    self.lon.plan_source = source
    self.lon.plan_accel = float(plan)
    self.lon.plan_accel_next = float(plan)
    self.lon.plan_curve_accel = float(plan)
    self.lon.plan_lead_accel = float(plan)
    self.lon.command = LongitudinalCommand(
      enabled, False, self.cs.out.vEgo,
      bool(base_accel), not bool(base_accel),
      base_state, 0.0, 200,
    )
    self.cs.out.cruiseState.enabled = bool(enabled)
    self.cs.out.brakePressed = bool(brake_pressed)
    self.cs.out.gasPressed = bool(gas_pressed)
    self.cs.out.aEgo = float(measured_accel)
    self.cs.hybrid_brake_request_raw_275 = int(feedback)
    self.cs.hybrid_friction_raw_08c = int(friction)
    for suffix in ("275", "08c", "2c9", "12a", "125"):
      setattr(self.cs, "hybrid_feedback_rx_frame_" + suffix,
              frame if feedback_fresh else -1000000)
    _, sends = self.car.update(
      enabled, enabled, self.cs, frame, FakeActuators(pid),
      not enabled, 0, False, False, False, False, False, None,
    )
    found = {}
    for msg in sends:
      # DragonPilot 0.8.13 packer returns [address, bus_time, bytes, bus].
      self.assertEqual(len(msg), 4)
      addr, _, raw, bus = msg
      if addr not in (0x271, 0x273):
        continue
      data = bytes(raw)
      self.assertEqual(bus, 0)
      self.assertEqual(len(data), 8)
      self.assertEqual(data[-1], dnga_checksum(addr, data[:-1]))
      found[addr] = data
    self.assertEqual(set(found), {0x271, 0x273})
    return {
      "state": found[0x271][1],
      "mag": int.from_bytes(found[0x271][4:6], "big"),
      "pump_raw": (found[0x271][3] - 256
                   if found[0x271][3] > 127 else found[0x271][3]),
      "target_centikph": int.from_bytes(found[0x273][2:4], "big"),
      "is_decel": bool(found[0x273][4] & 0x20),
      "is_accel": bool(found[0x273][4] & 0x40),
    }

  def test_braking_ready_24_frame_release_acceleration(self):
    # The V5.0 fixture remains READY; this verifies R2 actually alters CAN.
    for frame in (5, 10, 15, 20):
      out = self.tick(frame, feedback=(-164 if frame > 5 else 0))
      self.assertEqual(out["state"], BrakeState.BRAKING)
      self.assertEqual(out["pump_raw"], -4)
      self.assertLess(out["target_centikph"], 7200)
      self.assertTrue(out["is_decel"])
      self.assertFalse(out["is_accel"])

    # Simulate a request to resume: negative 0x275 and friction are gone.
    # During the 24 release frames, 0x271 pump stays -0.4 and ACCEL is vetoed.
    for frame in range(25, 145, 5):
      out = self.tick(frame, plan=0.25, pid=0.25, feedback=0)
      self.assertEqual(out["state"], BrakeState.READY, "frame {}".format(frame))
      self.assertEqual(out["mag"], 1224, "frame {}".format(frame))
      self.assertEqual(out["pump_raw"], -4, "frame {}".format(frame))
      self.assertFalse(out["is_accel"], "frame {}".format(frame))

    # In real V5.0 this latch is cleared by its handoff state machine.
    # The fixture explicitly models this normal readiness transition.
    self.lon.handoff_pending = False
    out = self.tick(145, plan=0.25, pid=0.25, feedback=0,
                    measured_accel=0.05)
    self.assertEqual(out["state"], BrakeState.READY)
    self.assertEqual(out["mag"], 200)
    self.assertEqual(out["pump_raw"], 0)
    self.assertTrue(out["is_accel"])

  def test_override_clears_new_r2_braking(self):
    first = self.tick(5)
    self.assertEqual(first["state"], BrakeState.BRAKING)
    with_pedal = self.tick(10, brake_pressed=True, base_accel=False)
    self.assertEqual(with_pedal["state"], BrakeState.READY)
    self.assertFalse(with_pedal["is_accel"])
    self.assertEqual(self.car.r2_v4_shadow_errors, 0)

    self.cs.out.brakePressed = False
    self.tick(15)
    cancelled = self.tick(20, enabled=False, base_state=BrakeState.DISABLED,
                          base_accel=False)
    self.assertEqual(cancelled["state"], BrakeState.DISABLED)
    self.assertFalse(cancelled["is_accel"])

  def test_missing_275_response_is_detected_but_not_a_safety_exit(self):
    # This explicitly documents an unresolved implementation hazard:
    # the observer detects missing OEM-style 0x275 response after three
    # distinct feedback frames, but the live R2 actuator does not act
    # on the detection. Do not treat this test as a safety qualification.
    last = None
    for frame in (5, 10, 15, 20):
      last = self.tick(frame, feedback=0, friction=0)
    self.assertTrue(self.car.r2_v4_last.missing_275_response)
    self.assertEqual(last["state"], BrakeState.BRAKING)
    self.assertEqual(self.car.r2_v4_shadow_errors, 0)

  def test_stale_feedback_does_not_unlock_acceleration(self):
    self.tick(5)
    out = self.tick(10, plan=0.3, pid=0.3,
                    feedback_fresh=False, base_accel=True)
    self.assertFalse(out["is_accel"])
    self.assertIn("stale_feedback", self.car.r2_v4_last.reason)


if __name__ == "__main__":
  unittest.main(verbosity=2)
