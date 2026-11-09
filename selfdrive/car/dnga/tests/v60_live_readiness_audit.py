#!/usr/bin/env python3
"""Offline DNGA v6.0 live-braking readiness audit.

This is intentionally *not* a vehicle activation or ECU simulator.
It exercises the real integrated R2 decision and checks known hazard paths.
A failed exit status means the software is NOT justified as road-ready.
"""
import json
import sys
from pathlib import Path
from types import SimpleNamespace

# Reuse the same non-actuating cereal/CAN fixtures as the v6 CI.
import test_v60_integrated_r2 as fixture


def setup():
  x = fixture.TestV60IntegratedR2()
  x.setUp()
  return x.ctrl, x.cs


def raw_feedback(cs, frame, request=0, friction=0):
  cs.hybrid_feedback_rx_frame_275 = frame
  cs.hybrid_feedback_rx_frame_08c = frame
  cs.hybrid_brake_request_raw_275 = request
  cs.hybrid_friction_raw_08c = friction


def evaluate():
  fixture.TestV60IntegratedR2.setUpClass()
  results = {}
  ctrl, cs = setup()
  first = ctrl.update(True, cs, 5, -0.20, False, False)
  results["baseline_1216_entry"] = {
    "observed": first.magnitude,
    "pass": ctrl.last_authority == "r2_braking" and first.magnitude == 1216
            and first.speed < cs.out.vEgo,
    "detail": "Existing isolated integration tests verify first magnitude is 1216 and ACC target below vEgo",
  }

  ctrl, cs = setup()
  cs.out.vEgo = 6.0
  raw_feedback(cs, 5, request=-163)
  before = ctrl.update(True, cs, 5, -0.20, False, False)
  cs.out.vEgo = 5.5
  raw_feedback(cs, 10, request=-163)
  after = ctrl.update(True, cs, 10, -0.20, False, True)
  results["moving_brake_to_sng"] = {
    "pass": ctrl.last_authority == "legacy_sng",
    "before_state": before.brake_state,
    "after_state": after.brake_state,
    "after_authority": ctrl.last_authority,
    "legacy_update_calls": ctrl.low_speed.calls,
    "detail": "An R2-owned braking session must transition to a verified low-speed stop/hold path without dropping brake authority",
  }

  ctrl, cs = setup()
  ctrl.plan_sm.msg = SimpleNamespace(
    longitudinalPlanSource="turn", hasLead=False, accels=[-1.2] * 13,
  )
  raw_feedback(cs, 5, request=-163)
  urgent = ctrl.update(True, cs, 5, -1.2, False, False)
  results["first_urgent_brake"] = {
    "pass": False,  # No calibrated emergency-stop response/backup demonstrated.
    "first_magnitude": urgent.magnitude,
    "first_pump": urgent.pump,
    "detail": "First emergency-demand frame is not validated against a safe stop envelope. No independent emergency authority is exercised here",
  }

  ctrl, cs = setup()
  last = None
  for frame in (5, 10, 15, 20):
    raw_feedback(cs, frame, request=0)
    last = ctrl.update(True, cs, frame, -0.20, False, False)
  results["missing_275_brake_response"] = {
    "pass": False,  # Correct recovery requires physical/ECU validation.
    "observed_fault": ctrl.last_feedback_fault,
    "command_still_braking": last.brake_state == 0x21,
    "detail": "Repeated fresh nonnegative 0x275 feedback is flagged but does not enforce a validated degraded/braking fallback",
  }

  code = (Path(__file__).resolve().parents[1] / "carcontroller.py").read_text()
  results["live_can_gate"] = {
    "pass": "V6_R2_ACTIVE_CAN = False" in code,
    "active": "V6_R2_ACTIVE_CAN = True" in code,
    "detail": "CAN remains default-disabled; do not enable until outstanding blockers are independently validated",
  }

  blockers = [key for key, value in results.items() if not value["pass"]]
  print(json.dumps({
    "purpose": "diagnostic/preflight only; no CAN, Panda, or real brake actuation",
    "live_ready": len(blockers) == 0,
    "blockers": blockers,
    "checks": results,
  }, indent=2))
  return 0 if not blockers else 2


if __name__ == "__main__":
  sys.exit(evaluate())
