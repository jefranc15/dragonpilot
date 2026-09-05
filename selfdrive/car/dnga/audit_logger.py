#!/usr/bin/env python3
"""Read-only DNGA longitudinal handoff audit logger for road tests.

Subscribes to carState/carControl/controlsState/longitudinalPlan/radarState and
raw can/sendcan. It publishes nothing and cannot alter vehicle control.
"""

import argparse
import csv
import os
import signal
import subprocess
import sys
import time

import cereal.messaging as messaging


HYBRID_IDS = (0x08C, 0x125, 0x12A, 0x275, 0x2C9)
RUNNING = True


def stop(_sig, _frame):
  global RUNNING
  RUNNING = False


def signed(raw, bits):
  sign = 1 << (bits - 1)
  return raw - (1 << bits) if raw & sign else raw


def age_ms(now_ns, item):
  return "" if not item else max(0.0, (now_ns - item[0]) / 1e6)


def get(obj, name, default=""):
  try:
    return getattr(obj, name)
  except Exception:
    return default


def as_float(value):
  try:
    return float(value)
  except Exception:
    return ""


def as_bool(value):
  try:
    return int(bool(value))
  except Exception:
    return ""


def git_value(*args):
  try:
    return subprocess.check_output(args, cwd="/data/openpilot", stderr=subprocess.DEVNULL, text=True).strip()
  except Exception:
    return "unknown"


def toyota_checksum(addr, dat):
  if len(dat) < 2:
    return 0
  expected = ((addr & 0xFF) + ((addr >> 8) & 0xFF) + len(dat) + sum(dat[:-1])) & 0xFF
  return int(dat[-1] == expected)


def dnga_checksum(addr, dat):
  if len(dat) != 8:
    return 0
  return int(dat[-1] == ((addr + len(dat) + 2 + sum(dat[:-1])) & 0xFF))


def keep(raw, key, mono_ns, msg):
  raw[key] = (int(mono_ns), bytes(msg.dat), int(msg.src))


def drain_can(sock, raw, outgoing=False):
  count = 0
  for evt in messaging.drain_sock(sock, wait_for_one=False):
    mono_ns = int(evt.logMonoTime)
    msgs = evt.sendcan if outgoing else evt.can
    for msg in msgs:
      count += 1
      addr, src = int(msg.address), int(msg.src)
      if outgoing:
        if addr == 0x271:
          keep(raw, "op271", mono_ns, msg)
        elif addr == 0x273:
          keep(raw, "op273", mono_ns, msg)
      elif src == 1 and addr in HYBRID_IDS:
        keep(raw, "h%03x" % addr, mono_ns, msg)
      elif src == 1 and addr == 0x277:
        keep(raw, "gas277", mono_ns, msg)
      elif src == 1 and addr == 0x037:
        keep(raw, "rpm037", mono_ns, msg)
      elif src == 2 and addr == 0x271:
        keep(raw, "stock271", mono_ns, msg)
      elif src == 2 and addr == 0x273:
        keep(raw, "stock273", mono_ns, msg)
  return count


def decode_271(prefix, item, now_ns):
  out = {prefix + k: "" for k in ("age_ms", "checksum", "state", "pump_reaction", "pump_level", "magnitude", "decel", "raw")}
  if not item:
    return out
  dat = item[1]
  out[prefix + "age_ms"] = age_ms(now_ns, item)
  out[prefix + "raw"] = dat.hex()
  if len(dat) == 8:
    out[prefix + "checksum"] = dnga_checksum(0x271, dat)
    out[prefix + "state"] = dat[1]
    out[prefix + "pump_reaction"] = signed(dat[3], 8) / 10.0
    out[prefix + "pump_level"] = dat[4] / 10.0
    out[prefix + "magnitude"] = dat[5]
    out[prefix + "decel"] = max(0.0, min(2.0, (200 - dat[5]) / 100.0))
  return out


def decode_273(prefix, item, now_ns):
  out = {prefix + k: "" for k in ("age_ms", "checksum", "enabled", "lead", "is_accel", "is_decel", "acc_cmd_kph", "set_speed_kph", "raw")}
  if not item:
    return out
  dat = item[1]
  out[prefix + "age_ms"] = age_ms(now_ns, item)
  out[prefix + "raw"] = dat.hex()
  if len(dat) == 8:
    out[prefix + "checksum"] = dnga_checksum(0x273, dat)
    out[prefix + "enabled"] = int(bool(dat[1] & 0x20))
    out[prefix + "lead"] = int(bool(dat[1] & 0x08))
    out[prefix + "is_accel"] = int(bool(dat[4] & 0x40))
    out[prefix + "is_decel"] = int(bool(dat[4] & 0x20))
    out[prefix + "acc_cmd_kph"] = ((dat[2] << 8) | dat[3]) * 0.01
    out[prefix + "set_speed_kph"] = dat[0]
  return out


def hybrid(raw, now_ns):
  names = {0x275: "275", 0x2C9: "2c9", 0x12A: "12a", 0x125: "125", 0x08C: "08c"}
  out = {}
  vals = {"brake": "", "req": "", "actual": "", "t12a": "", "t125": "", "friction": ""}
  ages = []

  for addr in HYBRID_IDS:
    suffix = names[addr]
    item = raw.get("h%03x" % addr)
    out["h_%s_age_ms" % suffix] = age_ms(now_ns, item)
    out["h_%s_checksum" % suffix] = ""
    out["h_%s_raw" % suffix] = "" if not item else item[1].hex()
    if not item:
      ages.append(999999.0)
      continue
    dat = item[1]
    ages.append(out["h_%s_age_ms" % suffix])
    out["h_%s_checksum" % suffix] = toyota_checksum(addr, dat)
    if not out["h_%s_checksum" % suffix]:
      continue
    if addr == 0x275 and len(dat) == 8:
      vals["brake"] = signed(int.from_bytes(dat[0:2], "big"), 16)
      vals["req"] = signed(int.from_bytes(dat[2:4], "big"), 16)
    elif addr == 0x2C9 and len(dat) == 8:
      vals["actual"] = signed(int.from_bytes(dat[5:7], "big"), 16)
    elif addr == 0x12A and len(dat) == 7:
      vals["t12a"] = signed(int.from_bytes(dat[2:4], "big") & 0x7FF, 11)
    elif addr == 0x125 and len(dat) == 7:
      vals["t125"] = signed(int.from_bytes(dat[3:5], "big") & 0x7FF, 11)
    elif addr == 0x08C and len(dat) == 8:
      vals["friction"] = dat[2]

  complete = all(v != "" for v in vals.values())
  fresh = complete and all(0.0 <= a <= 250.0 for a in ages)
  consistent = brakes_clear = torque_ready = positive_vote = False
  e_actual_12a = e_dup = ""
  if complete:
    e_actual_12a = abs((vals["actual"] * 6 - vals["t12a"] * 73) / 6.0)
    e_dup = abs(vals["t12a"] - vals["t125"])
    consistent = e_actual_12a <= 400.0 and e_dup <= 100
    brakes_clear = vals["brake"] >= -100 and vals["friction"] <= 0
    torque_ready = vals["req"] >= -100 and vals["actual"] >= -100 and vals["t12a"] >= -8 and vals["t125"] >= -8
    positive_vote = vals["req"] > 80 and vals["actual"] > 80 and vals["t12a"] > 5 and vals["t125"] > 5

  out.update({
    "h_brake_request_275": vals["brake"], "h_torque_request_275": vals["req"],
    "h_torque_actual_2c9": vals["actual"], "h_torque_12a": vals["t12a"],
    "h_torque_125": vals["t125"], "h_friction_08c": vals["friction"],
    "h_fresh": int(fresh), "h_consistent": int(consistent),
    "h_feedback_clean": int(fresh and consistent), "h_brakes_clear": int(brakes_clear),
    "h_torque_ramp_ready": int(torque_ready), "h_positive_vote": int(positive_vote),
    "h_physical_ready": int(fresh and consistent and brakes_clear and torque_ready),
    "h_actual_12a_error": e_actual_12a, "h_duplicate_error": e_dup,
  })
  return out


def row_from(sm, raw, index, rx_count, tx_count):
  now_ns = time.monotonic_ns()
  cs, cc = sm["carState"], sm["carControl"]
  controls, plan, radar = sm["controlsState"], sm["longitudinalPlan"], sm["radarState"]
  cruise = get(cs, "cruiseState", None)
  act = get(cc, "actuators", None)
  lead0, lead1 = get(radar, "leadOne", None), get(radar, "leadTwo", None)

  try:
    accels = get(plan, "accels", [])
    plan0 = float(accels[0]) if len(accels) else ""
    plan1 = float(accels[1]) if len(accels) > 1 else plan0
  except Exception:
    plan0 = plan1 = ""
  actuator_accel = as_float(get(act, "accel", ""))
  positive_agree = int(isinstance(plan0, float) and isinstance(actuator_accel, float) and plan0 >= 0.05 and actuator_accel >= 0.05)

  out = {
    "row": index, "unix_time": time.time(), "mono_ns": now_ns,
    "v_ego": as_float(get(cs, "vEgo", "")), "a_ego": as_float(get(cs, "aEgo", "")),
    "standstill": as_bool(get(cs, "standstill", "")), "gas_pressed": as_bool(get(cs, "gasPressed", "")),
    "brake_pressed": as_bool(get(cs, "brakePressed", "")), "engine_rpm": as_float(get(cs, "engineRPM", "")),
    "cruise_available": as_bool(get(cruise, "available", "")), "cruise_enabled": as_bool(get(cruise, "enabled", "")),
    "cruise_speed": as_float(get(cruise, "speed", "")), "distance_lines": get(cs, "distanceLines", ""),
    "carcontrol_enabled": as_bool(get(cc, "enabled", "")), "carcontrol_active": as_bool(get(cc, "active", "")),
    "actuator_accel": actuator_accel, "long_control_state": str(get(controls, "longControlState", "")),
    "plan_age_ms": age_ms(now_ns, (int(sm.logMonoTime.get("longitudinalPlan", 0)), b"", 0)) if sm.logMonoTime.get("longitudinalPlan", 0) else "",
    "plan_source": str(get(plan, "longitudinalPlanSource", "")), "plan_has_lead": as_bool(get(plan, "hasLead", "")),
    "plan_accel0": plan0, "plan_accel1": plan1,
    "radar_age_ms": age_ms(now_ns, (int(sm.logMonoTime.get("radarState", 0)), b"", 0)) if sm.logMonoTime.get("radarState", 0) else "",
    "lead0_status": as_bool(get(lead0, "status", "")), "lead0_drel": as_float(get(lead0, "dRel", "")),
    "lead0_vrel": as_float(get(lead0, "vRel", "")), "lead0_vlead": as_float(get(lead0, "vLead", "")),
    "lead1_status": as_bool(get(lead1, "status", "")), "lead1_drel": as_float(get(lead1, "dRel", "")),
    "lead1_vrel": as_float(get(lead1, "vRel", "")), "lead1_vlead": as_float(get(lead1, "vLead", "")),
    "positive_planner_pid_agreement": positive_agree,
    "rx_can_messages": rx_count, "tx_can_messages": tx_count,
  }

  gas = raw.get("gas277")
  gas_raw = int.from_bytes(gas[1][1:3], "big") if gas and len(gas[1]) >= 3 else ""
  rpm = raw.get("rpm037")
  rpm_raw = max(0, signed(int.from_bytes(rpm[1][3:5], "big"), 16)) if rpm and len(rpm[1]) >= 5 else ""
  out.update({
    "gas277_age_ms": age_ms(now_ns, gas), "gas277_raw": gas_raw,
    "gas277_pressed": int(gas_raw > 100) if gas_raw != "" else "",
    "rpm037_age_ms": age_ms(now_ns, rpm), "rpm037": rpm_raw,
  })

  out.update(hybrid(raw, now_ns))
  out["observed_ready_to_ramp"] = int(bool(out["h_physical_ready"] and positive_agree))
  out.update(decode_271("op271_", raw.get("op271"), now_ns))
  out.update(decode_273("op273_", raw.get("op273"), now_ns))
  out.update(decode_271("stock271_", raw.get("stock271"), now_ns))
  out.update(decode_273("stock273_", raw.get("stock273"), now_ns))
  return out


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--output-dir", default="/data/dnga_audit")
  ap.add_argument("--sample-hz", type=float, default=20.0)
  args = ap.parse_args()
  os.makedirs(args.output_dir, exist_ok=True)

  stamp = time.strftime("%Y%m%d_%H%M%S")
  csv_path = os.path.join(args.output_dir, "dnga_audit_%s.csv" % stamp)
  meta_path = os.path.join(args.output_dir, "dnga_audit_%s.meta.txt" % stamp)
  branch, commit = git_value("git", "branch", "--show-current"), git_value("git", "rev-parse", "HEAD")
  status = git_value("git", "status", "--short")
  with open(meta_path, "w") as f:
    f.write("branch=%s\ncommit=%s\nstarted_unix=%.6f\ngit_status_begin\n%s\ngit_status_end\n" % (branch, commit, time.time(), status))

  services = ["carState", "carControl", "controlsState", "longitudinalPlan", "radarState"]
  sm = messaging.SubMaster(services, poll=["carState"], ignore_alive=services, ignore_avg_freq=services)
  can_sock = messaging.sub_sock("can", conflate=False)
  sendcan_sock = messaging.sub_sock("sendcan", conflate=False)

  raw, index = {}, 0
  last_sample, last_flush, last_report = 0.0, time.monotonic(), time.monotonic()
  sample_period = 1.0 / max(1.0, args.sample_hz)
  total_rx = total_tx = 0
  writer = None

  print("DNGA PASSIVE logger: branch=%s commit=%s" % (branch, commit))
  print("CSV: %s" % csv_path)
  print("META: %s" % meta_path)
  sys.stdout.flush()

  with open(csv_path, "w", newline="") as f:
    while RUNNING:
      sm.update(1000)
      rx = drain_can(can_sock, raw, outgoing=False)
      tx = drain_can(sendcan_sock, raw, outgoing=True)
      total_rx += rx
      total_tx += tx
      if not sm.updated.get("carState", False):
        continue
      now = time.monotonic()
      if last_sample and now - last_sample < sample_period:
        continue
      last_sample = now
      row = row_from(sm, raw, index, rx, tx)
      if writer is None:
        writer = csv.DictWriter(f, fieldnames=list(row.keys()))
        writer.writeheader()
      writer.writerow(row)
      index += 1
      if now - last_flush >= 1.0:
        f.flush()
        last_flush = now
      if now - last_report >= 60.0:
        print("rows=%d rx=%d tx=%d" % (index, total_rx, total_tx))
        sys.stdout.flush()
        last_report = now
    f.flush()

  print("stopped rows=%d csv=%s" % (index, csv_path))


if __name__ == "__main__":
  signal.signal(signal.SIGINT, stop)
  signal.signal(signal.SIGTERM, stop)
  main()
