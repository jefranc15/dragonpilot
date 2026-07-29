#!/usr/bin/env python3
from __future__ import print_function

import os
import py_compile
import shutil

PATH = "/data/openpilot/selfdrive/car/dnga/carcontroller.py"
BACKUP = PATH + ".before_brake_logger"
IMPORT_MARKER = "# DNGA_BRAKE_DEBUG_IMPORTS"
INIT_MARKER = "# DNGA_BRAKE_DEBUG_INIT_BEGIN"
HOOK_MARKER = "# DNGA_BRAKE_DEBUG_HOOK_BEGIN"


def insert_imports(text):
  if IMPORT_MARKER in text:
    return text

  lines = text.splitlines(True)
  index = 0

  if lines and lines[0].startswith("#!"):
    index = 1

  while index < len(lines) and lines[index].startswith("from __future__ import"):
    index += 1

  block = (
    IMPORT_MARKER + "\n"
    "import json\n"
    "import socket\n"
  )
  lines.insert(index, block)
  return "".join(lines)


def insert_init(text):
  if INIT_MARKER in text:
    return text

  update_pos = text.find("\n  def update(")
  if update_pos < 0:
    raise RuntimeError("Could not locate CarController.update().")

  before_update = text[:update_pos]
  target = "    self.brake_active = False"
  init_pos = before_update.rfind(target)

  if init_pos < 0:
    raise RuntimeError("Could not locate self.brake_active initialization.")

  line_end = text.find("\n", init_pos)
  if line_end < 0:
    line_end = len(text)

  block = '''

    # DNGA_BRAKE_DEBUG_INIT_BEGIN
    try:
      self._dnga_brake_debug_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
      self._dnga_brake_debug_sock.setblocking(False)
    except Exception:
      self._dnga_brake_debug_sock = None
    # DNGA_BRAKE_DEBUG_INIT_END'''

  return text[:line_end] + block + text[line_end:]


def insert_hook(text):
  if HOOK_MARKER in text:
    return text

  target = "      brake_amt_for_hud = clip(brake_request, 0.0, 0.60) if decel_req else 0.0"
  pos = text.find(target)

  if pos < 0:
    raise RuntimeError(
      "Could not locate the v2.5c brake_amt_for_hud line. "
      "This installer is intended for the v2.5c brake-only CarController."
    )

  line_end = text.find("\n", pos)
  if line_end < 0:
    line_end = len(text)

  block = r'''

      # DNGA_BRAKE_DEBUG_HOOK_BEGIN
      try:
        _brake_debug = {
          "frame": int(frame),
          "enabled": bool(enabled),
          "active": bool(active),
          "cruiseEnabled": bool(CS.out.cruiseState.enabled),
          "leadArg": bool(lead),
          "vEgo": float(CS.out.vEgo),
          "aEgo": float(CS.out.aEgo),
          "gasPressed": bool(CS.out.gasPressed),
          "brakePressed": bool(CS.out.brakePressed),
          "standstill": bool(CS.out.standstill),
          "cruiseSetSpeed": float(set_speed),
          "actuatorAccel": float(actuators.accel),
          "actuatorSpeed": float(getattr(actuators, "speed", 0.0)),
          "applyAccel": float(apply_accel),
          "applyBrake": float(apply_brake),
          "desSpeed": float(des_speed),
          "baseSpeedForLead": float(base_speed_for_lead),
          "targetSpeedForLead": float(target_speed_for_lead),
          "leadSpeedError": float(lead_speed_error),
          "leadAssistOK": bool(lead_assist_ok),
          "leadAssistBrake": float(lead_assist_brake),
          "overspeed": float(overspeed),
          "overspeedBrake": float(overspeed_brake),
          "brakeRequest": float(brake_request),
          "leadJustAppeared": bool(lead_just_appeared),
          "cutInSoftActive": bool(cut_in_soft_active),
          "cutInBrakeOK": bool(cut_in_brake_ok),
          "leadAssistActive": bool(lead_assist_active),
          "accelGuardOK": bool(accel_guard_ok),
          "brakeAllowed": bool(brake_allowed),
          "brakeReqCounter": int(self.brake_req_counter),
          "brakeReleaseCounter": int(self.brake_release_counter),
          "brakeActive": bool(self.brake_active),
          "decelReq": bool(decel_req),
          "brakeState": int(brake_state),
          "pumpReaction": float(pump_reaction),
          "brakeMag": int(brake_mag),
          "brakeAmtForHud": float(brake_amt_for_hud),
          "blockBrakeUntilFrame": int(self.block_brake_until_frame),
          "framesUntilBrakeAllowed": int(max(0, self.block_brake_until_frame - frame)),
          "cutInSoftUntilFrame": int(getattr(self, "cut_in_soft_until_frame", 0)),
        }

        if self._dnga_brake_debug_sock is not None:
          self._dnga_brake_debug_sock.sendto(
            json.dumps(_brake_debug, separators=(",", ":")).encode("utf-8"),
            ("127.0.0.1", 8062)
          )
      except Exception:
        pass
      # DNGA_BRAKE_DEBUG_HOOK_END'''

  return text[:line_end] + block + text[line_end:]


def main():
  if not os.path.isfile(PATH):
    raise SystemExit("Not found: " + PATH)

  with open(PATH, "r") as f:
    original = f.read()

  if HOOK_MARKER in original and INIT_MARKER in original:
    print("DNGA brake debug hook is already installed.")
    return

  if not os.path.exists(BACKUP):
    shutil.copy2(PATH, BACKUP)
    print("Backup created:", BACKUP)
  else:
    print("Using existing backup:", BACKUP)

  try:
    updated = insert_imports(original)
    updated = insert_init(updated)
    updated = insert_hook(updated)

    with open(PATH, "w") as f:
      f.write(updated)

    py_compile.compile(PATH, doraise=True)
  except Exception as exc:
    shutil.copy2(BACKUP, PATH)
    raise SystemExit("Install failed; original restored: %s" % exc)

  print("Installed DNGA v2.5c brake debug hook.")
  print("CarController compile passed.")


if __name__ == "__main__":
  main()
