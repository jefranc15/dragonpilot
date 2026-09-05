"""Offline differential checks against the original nightly commit.

Run from the repository root: python selfdrive/car/dnga/tests/compare_behavior.py
Requires cantools for DBC packing. Messaging and vehicle I/O are simulated;
both implementations execute their real controller and CAN encoder functions.
"""

import ast
import copy
import importlib.util
import random
import subprocess
import sys
import types
import unittest
from collections import Counter, defaultdict
from pathlib import Path

import cantools

ROOT = Path(__file__).resolve().parents[4]
BASE = "12c70379a1661c4783df10fe5730a45258567a72"
CAR = "TOYOTA DNGA YARIS CROSS HEV AC200"
FEEDS = {}
UPDATES = {}
FAILURES = set()
FEATURES = set()
DATABASE = cantools.database.load_file(ROOT / "opendbc/dnga_hev.dbc", strict=False)


def module(name, **attrs):
  result = types.ModuleType(name)
  result.__dict__.update(attrs)
  sys.modules[name] = result
  return result


class Packer:
  """Capture unquantized signals as well as packed bytes for comparison."""

  def __init__(self, dbc):
    self.calls = []

  def make_can_msg(self, name, bus, values):
    message = DATABASE.get_message_by_name(name)
    self.calls.append((name, bus, copy.deepcopy(values)))
    signals = {signal.name: signal.offset for signal in message.signals}
    signals.update(values)
    return [message.frame_id, 0, message.encode(signals, strict=False), bus]


class SubMaster:
  def __init__(self, services):
    self.service = services[0]
    if self.service in FAILURES:
      raise RuntimeError("simulated subscriber creation failure")
    self.updated = {self.service: False}

  def update(self, timeout):
    if self.service in FAILURES:
      raise RuntimeError("simulated subscriber read failure")
    self.updated[self.service] = UPDATES.get(self.service, True)

  def __getitem__(self, service):
    return FEEDS[service]


class Features:
  def has(self, feature):
    return feature in FEATURES


class Actuators:
  def __init__(self, accel=0.0, steer=0.0):
    self.accel = accel
    self.steer = steer

  def copy(self):
    return copy.copy(self)


module("common")
spec = importlib.util.spec_from_file_location("common.numpy_fast", ROOT / "common/numpy_fast.py")
numpy_fast = importlib.util.module_from_spec(spec)
spec.loader.exec_module(numpy_fast)
sys.modules[spec.name] = numpy_fast
module("common.features", Features=Features)
module("selfdrive")
module(
  "selfdrive.car",
  make_can_msg=lambda addr, dat, bus: [addr, 0, dat, bus],
  dbc_dict=lambda pt, radar: {"pt": pt, "radar": radar},
)
module("selfdrive.config", Conversions=types.SimpleNamespace(KPH_TO_MS=1.0 / 3.6, MS_TO_KPH=3.6))
module("opendbc")
module("opendbc.can")
module("opendbc.can.packer", CANPacker=Packer)
module(
  "cereal",
  car=types.SimpleNamespace(CarParams=types.SimpleNamespace(Ecu=types.SimpleNamespace(fwdCamera=1))),
  messaging=types.SimpleNamespace(SubMaster=SubMaster),
)


def load_package(name, baseline=False):
  package = module(name)
  package.__path__ = []
  files = ["values", "dngacan", "dnga_hybrid_feedback"]
  if not baseline:
    files.append("longitudinal")
  files.append("carcontroller")
  for filename in files:
    path = ROOT / f"selfdrive/car/dnga/{filename}.py"
    if baseline:
      source = subprocess.check_output(["git", "show", f"{BASE}:{path.relative_to(ROOT)}"], cwd=ROOT, text=True)
      path = Path("baseline") / filename
    else:
      source = path.read_text()
    child = module(f"{name}.{filename}")
    child.__file__ = str(path)
    exec(compile(source.replace("selfdrive.car.dnga", name), str(path), "exec"), child.__dict__)
    setattr(package, filename, child)
  return package


BEFORE = load_package("dnga_before", baseline=True)
AFTER = load_package("dnga_after")
BASE_REARM_FIELD = next(
  value
  for value in BEFORE.carcontroller.CarController.update.__code__.co_consts
  if isinstance(value, str) and value.endswith("acc_rearm_edge")
)
CP = types.SimpleNamespace(
  carFingerprint=CAR,
  lateralParams=types.SimpleNamespace(torqueBP=[0.0, 10.0, 20.0, 35.0], torqueV=[255, 255, 255, 255]),
)


def state():
  result = types.SimpleNamespace(
    out=types.SimpleNamespace(
      vEgo=15.0,
      aEgo=0.0,
      standstill=False,
      gasPressed=False,
      brakePressed=False,
      leftBlinker=False,
      rightBlinker=False,
      steeringPressed=False,
      steeringTorqueEps=0.0,
      cruiseState=types.SimpleNamespace(enabled=True, available=True),
    ),
    cruise_speed=20.0,
    lkas_latch=True,
    op_distance_val=1,
    stock_fcw=False,
    stock_aeb=False,
    stock_adas_frontDepartureHUD=False,
    stock_lkc_off=False,
    stock_fcw_off=False,
    stock_acc_brake_state=0,
    stock_acc_brake_decel=0.0,
    stock_acc_brake_rx_frame=-1000000,
    stock_acc_request_enabled=False,
    stock_acc_request_lead=False,
    stock_acc_request_is_accel=False,
    stock_acc_request_is_decel=False,
    stock_acc_request_rx_frame=-1000000,
    is_cruise_latch=True,
    hybrid_feedback_fault=False,
    hybrid_feedback_fault_reason="",
  )
  BEFORE.dnga_hybrid_feedback.initialize_hybrid_feedback_state(result)
  return result


def configure(cs, frame, sample):
  cs.out.vEgo = sample.get("speed", 15.0)
  cs.out.aEgo = sample.get("aego", 0.0)
  cs.out.standstill = sample.get("standstill", cs.out.vEgo < 0.01)
  cs.out.gasPressed = sample.get("gas", False)
  cs.out.brakePressed = sample.get("pedal_brake", False)
  cs.out.cruiseState.enabled = sample.get("cruise", True)
  cs.out.cruiseState.available = sample.get("available", True)
  cs.lkas_latch = sample.get("lkas", True)
  cs.op_distance_val = sample.get("distance", 1)
  cs.out.leftBlinker = sample.get("left_blinker", False)
  cs.out.rightBlinker = sample.get("right_blinker", False)
  cs.out.steeringPressed = sample.get("driver_steering", False)
  cs.out.steeringTorqueEps = sample.get("torque", 0.0)
  cs.acc_rearm_edge = sample.get("rearm", False)
  setattr(cs, BASE_REARM_FIELD, cs.acc_rearm_edge)
  torque = sample.get("hybrid_torque", 0)
  cs.hybrid_brake_request_raw_275 = sample.get("brake_feedback", 0)
  cs.hybrid_torque_request_raw_275 = sample.get("hybrid_request", torque)
  cs.hybrid_torque_actual_raw_2c9 = torque
  cs.hybrid_torque_raw_12a = sample.get("torque_12a", round(torque * 6 / 73))
  cs.hybrid_torque_raw_125 = sample.get("torque_125", round(torque * 6 / 73))
  cs.hybrid_friction_raw_08c = sample.get("friction", 0)
  for suffix in ("275", "2c9", "12a", "125", "08c"):
    setattr(cs, f"hybrid_feedback_rx_frame_{suffix}", frame - sample.get("feedback_age", 0))
  stock = sample.get("stock_brake", 0.0)
  cs.stock_acc_brake_decel = stock
  cs.stock_acc_brake_state = 0x21 if stock else 0x01
  cs.stock_acc_brake_rx_frame = frame - sample.get("stock_age", 0)
  cs.stock_acc_request_rx_frame = frame - sample.get("stock_age", 0)
  cs.stock_acc_request_enabled = bool(stock)
  cs.stock_acc_request_lead = bool(stock)
  cs.stock_acc_request_is_decel = bool(stock)
  cs.stock_acc_request_is_accel = False


def feeds(sample):
  lead = sample.get("lead", False)
  accel = sample.get("plan", sample.get("accel", 0.0))
  FEEDS["longitudinalPlan"] = types.SimpleNamespace(
    longitudinalPlanSource=sample.get("source", "lead0" if lead else "cruise"),
    hasLead=sample.get("has_lead", lead),
    accels=sample.get("accels", [accel, sample.get("plan_next", accel)]),
  )
  FEEDS["radarState"] = types.SimpleNamespace(
    leadOne=types.SimpleNamespace(
      status=sample.get("radar_lead", lead), dRel=sample.get("drel", 30.0), vRel=sample.get("vrel", 0.0)
    ),
    leadTwo=types.SimpleNamespace(
      status=sample.get("lead2", False), dRel=sample.get("drel2", 15.0), vRel=sample.get("vrel2", -1.0)
    ),
  )
  UPDATES["longitudinalPlan"] = sample.get("plan_update", True)
  UPDATES["radarState"] = sample.get("radar_update", True)
  FAILURES.clear()
  FAILURES.update(sample.get("failures", ()))


class TestControllerBehavior(unittest.TestCase):
  frames = 0
  messages = 0
  states = Counter()
  faults = Counter()

  def compare(self, samples, feature=False, constructor_failures=()):
    FEATURES.clear()
    if feature:
      FEATURES.add("ClearCode")
    FAILURES.clear()
    FAILURES.update(constructor_failures)
    old = BEFORE.carcontroller.CarController("dnga_hev", CP, None)
    new = AFTER.carcontroller.CarController("dnga_hev", CP, None)
    old_cs, new_cs = state(), state()
    for frame, sample in enumerate(samples):
      feeds(sample)
      for cs in (old_cs, new_cs):
        configure(cs, frame, sample)
      args = (
        frame,
        Actuators(sample.get("accel", 0.0), sample.get("steer", 0.3)),
        sample.get("cancel", False),
        0,
        sample.get("left_line", True),
        sample.get("right_line", True),
        sample.get("lead", False),
        False,
        False,
        None,
      )
      enabled = sample.get("enabled", frame >= 5)
      old_output = old.update(enabled, enabled, old_cs, *args)
      new_output = new.update(enabled, enabled, new_cs, *args)
      context = f"frame={frame}, sample={sample}"
      self.assertEqual(vars(old_output[0]), vars(new_output[0]), context)
      self.assertEqual(old_output[1], new_output[1], context)
      self.assertEqual(old.packer.calls, new.packer.calls, context)
      self.assertEqual(old.steer_rate_limited, new.steer_rate_limited, context)
      self.assertEqual(old_cs.hybrid_feedback_fault, new_cs.hybrid_feedback_fault, context)
      self.assertEqual(old_cs.hybrid_feedback_fault_reason, new_cs.hybrid_feedback_fault_reason, context)
      for name, bus, signals in new.packer.calls:
        if name == "ACC_BRAKE" and "CHECKSUM" in signals:
          self.states[signals["BRAKE_STATE"]] += 1
      if new_cs.hybrid_feedback_fault:
        self.faults[new_cs.hybrid_feedback_fault_reason] += 1
      old.packer.calls.clear()
      new.packer.calls.clear()
      type(self).frames += 1
      type(self).messages += len(new_output[1])

  def test_cruise_and_overrides(self):
    samples = []
    for distance in range(3):
      for speed in (0.0, 0.2, 1.0, 4.17, 8.49, 8.5, 15.0, 35.0):
        for extra in (
          {},
          {"gas": True},
          {"pedal_brake": True},
          {"cancel": True},
          {"enabled": False},
          {"cruise": False},
        ):
          samples.extend([dict(speed=speed, accel=0.3, distance=distance, **extra)] * 100)
    self.compare(samples)

  def test_braking_stop_hold_resume(self):
    samples = []
    for distance in range(3):
      samples += [dict(speed=10.0, accel=0.0, distance=distance)] * 100
      samples += [
        dict(speed=6.0, lead=True, drel=15.0, vrel=-2.0, accel=-0.7, stock_brake=0.87, aego=-0.4, distance=distance)
      ] * 180
      samples += [dict(speed=0.7, lead=True, drel=5.0, vrel=-0.7, accel=-0.25, distance=distance)] * 120
      samples += [dict(speed=0.0, lead=True, drel=5.0, vrel=0.0, accel=-0.1, distance=distance)] * 120
      samples += [dict(speed=0.2, standstill=False, lead=True, drel=5.0, vrel=-0.2, accel=-0.1, distance=distance)] * 70
      samples += [dict(speed=0.0, lead=True, drel=6.0, vrel=1.0, accel=0.3, distance=distance)] * 250
      samples += [dict(speed=2.0, lead=True, drel=10.0, vrel=1.0, accel=0.3, distance=distance)] * 250
    self.compare(samples)

  def test_lead_loss_and_highway_handoff(self):
    for speed in (0.8, 6.0, 8.0, 15.0, 30.0):
      samples = [dict(speed=speed, lead=True, drel=19.0, vrel=-3.0, accel=-0.8)] * 250
      samples += [dict(speed=speed, lead=True, source="cruise", drel=30.0, vrel=0.5, accel=0.3)] * 300
      samples += [dict(speed=speed, lead=True, drel=12.0, vrel=-2.0, accel=-0.5)] * 200
      samples += [dict(speed=speed, lead=False, accel=0.2)] * 250
      self.compare(samples)

  def test_feedback_faults_and_rearm(self):
    for fault in (
      {"feedback_age": 26},
      {"torque_12a": 900},
      {"torque_125": 200},
      {"hybrid_torque": 600, "friction": 1},
      {"feedback_age": -1},
    ):
      samples = [dict(accel=0.3)] * 150
      samples += [dict(accel=0.3, **fault)] * 100
      samples += [dict(accel=0.3)] * 100
      samples += [dict(accel=0.3, rearm=True, brake_feedback=-500)] * 10
      samples += [dict(accel=0.3, rearm=True, hybrid_torque=-150)] * 10
      samples += [dict(accel=0.3)] * 200
      self.compare(samples)

  def test_friction_overlap_timing(self):
    samples = [dict(lead=True, drel=15.0, vrel=-1.0, accel=-0.3, hybrid_torque=200)] * 150
    samples += [dict(lead=True, drel=15.0, vrel=-1.0, accel=-0.3, hybrid_torque=200, friction=1)] * 50
    samples += [dict(lead=True, drel=15.0, vrel=-1.0, accel=-0.3, hybrid_torque=0, friction=1)] * 30
    samples += [dict(lead=True, drel=15.0, vrel=-1.0, accel=-0.3, hybrid_torque=200, friction=1)] * 50
    samples += [dict(accel=0.3, rearm=True)] * 200
    self.compare(samples)

  def test_stale_and_malformed_subscribers(self):
    samples = [dict(accel=0.3)] * 100
    for extra in (
      {"plan_update": False},
      {"radar_update": False},
      {"accels": []},
      {"accels": [0.2]},
      {"source": "enum.LEAD1", "lead2": True},
      {"failures": ("longitudinalPlan", "radarState")},
    ):
      samples += [dict(lead=True, drel=12.0, vrel=-1.0, accel=-0.3, **extra)] * 150
    self.compare(samples)
    self.compare(samples, constructor_failures=("longitudinalPlan", "radarState"))

  def test_deterministic_transitions(self):
    rng = random.Random(813)
    samples = []
    for _ in range(500):
      sample = dict(
        speed=rng.choice([0.0, 0.01, 0.15, 0.5, 0.8, 1.0, 1.2, 2.5, 4.17, 8.0, 8.5, 15.0, 25.0, 35.0]),
        aego=rng.choice([-1.2, -0.9, -0.1, -0.02, 0.0, 0.25, 0.3]),
        accel=rng.choice([-3.5, -0.8, -0.45, -0.05, 0.0, 0.05, 0.3, 1.5]),
        plan=rng.choice([-0.8, -0.2, -0.03, 0.0, 0.05, 0.3]),
        lead=rng.random() < 0.75,
        drel=rng.choice([1.0, 5.0, 8.0, 12.0, 20.0, 35.0]),
        vrel=rng.choice([-3.0, -0.5, -0.05, 0.0, 0.4, 1.0]),
        distance=rng.randrange(3),
        gas=rng.random() < 0.06,
        pedal_brake=rng.random() < 0.04,
        cancel=rng.random() < 0.03,
        enabled=rng.random() < 0.95,
        steer=rng.uniform(-1.2, 1.2),
        torque=rng.uniform(-100, 100),
        left_blinker=rng.random() < 0.2,
        right_blinker=rng.random() < 0.2,
        stock_brake=rng.choice([0.0, 0.13, 0.36, 0.74, 0.75, 0.87]),
        stock_age=rng.choice([0, 25, 26, 50]),
        feedback_age=rng.choice([0, 0, 0, 25, 26]),
        rearm=rng.random() < 0.3,
        source=rng.choice(["cruise", "lead0", "lead1", "turn"]),
      )
      samples += [sample] * rng.randrange(5, 150)
    self.compare(samples, feature=True)


class TestFeedbackDecoder(unittest.TestCase):
  def test_payloads_and_freshness(self):
    rng = random.Random(275)
    old_cs, new_cs = state(), state()
    for frame in range(2500):
      addr = rng.choice([0x08C, 0x125, 0x12A, 0x275, 0x2C9, 0x271, 0x777])
      length = rng.choice([0, 1, 7, 8, 9])
      dat = bytearray(rng.getrandbits(8) for _ in range(length))
      if length and frame % 2 == 0:
        dat[-1] = ((addr & 255) + (addr >> 8) + length + sum(dat[:-1])) & 255
      old = BEFORE.dnga_hybrid_feedback
      new = AFTER.dnga_hybrid_feedback
      self.assertEqual(old.decode_hybrid_feedback_frame(addr, dat), new.decode_hybrid_feedback_frame(addr, dat))
      self.assertEqual(
        old.apply_hybrid_feedback_frame(old_cs, frame, addr, dat),
        new.apply_hybrid_feedback_frame(new_cs, frame, addr, dat),
      )
      self.assertEqual(old.hybrid_feedback_snapshot(old_cs, frame), new.hybrid_feedback_snapshot(new_cs, frame))


def original_source(filename):
  return subprocess.check_output(["git", "show", f"{BASE}:selfdrive/car/dnga/{filename}.py"], cwd=ROOT, text=True)


def normalize(value):
  if isinstance(value, types.SimpleNamespace):
    return {key: normalize(item) for key, item in vars(value).items()}
  if isinstance(value, (list, tuple)):
    return [normalize(item) for item in value]
  return value


class StateBase:
  def __init__(self, cp):
    self.CP = cp
    self.last_speed = 0.0

  def get_wheel_speeds(self, fl, fr, rl, rr):
    return types.SimpleNamespace(fl=fl, fr=fr, rl=rl, rr=rr)

  def update_speed_kf(self, speed):
    accel = (speed - self.last_speed) * 100
    self.last_speed = speed
    return speed, accel

  def parse_gear_shifter(self, gear):
    return gear


class CanDefine:
  def __init__(self, dbc):
    self.dv = {"TRANSMISSION": {"GEAR": {0: "P", 1: "D", 2: "R"}}, "ACC_CMD_HUD": {"FOLLOW_DISTANCE": {}}}


class Parser:
  def __init__(self, dbc, signals, checks, bus):
    self.config = (dbc, signals, checks, bus)
    self.vl = defaultdict(lambda: defaultdict(int))
    for signal, message, default in signals:
      self.vl[message][signal] = default


def load_state(name, source):
  car = sys.modules["cereal"].car
  car.CarState = types.SimpleNamespace(
    new_message=lambda: types.SimpleNamespace(cruiseState=types.SimpleNamespace(), buttonEvents=[]),
    ButtonEvent=types.SimpleNamespace(
      new_message=types.SimpleNamespace, Type=types.SimpleNamespace(gapAdjustCruise="gapAdjustCruise")
    ),
  )
  module("selfdrive.car.interfaces", CarStateBase=StateBase)
  module("opendbc.can.can_define", CANDefine=CanDefine)
  module("opendbc.can.parser", CANParser=Parser)
  result = module(name + ".carstate")
  exec(compile(source.replace("selfdrive.car.dnga", name), name + "/carstate.py", "exec"), result.__dict__)
  return result


class TestSupportingFiles(unittest.TestCase):
  def test_button_sequences(self):
    old_module = load_state("dnga_before", original_source("carstate"))
    new_module = load_state("dnga_after", (ROOT / "selfdrive/car/dnga/carstate.py").read_text())
    old_module.time = new_module.time = lambda: 1000.0
    cp = types.SimpleNamespace(carFingerprint=CAR, enableBsm=True)
    old, new = old_module.CarState(cp), new_module.CarState(cp)
    old_parser, new_parser = old.get_can_parser(cp), new.get_can_parser(cp)
    self.assertEqual(old_parser.config, new_parser.config)
    rng = random.Random(520)
    for frame in range(6000):
      old_module.time = new_module.time = lambda: 1000.0 + frame * 0.01
      updates = {
        "METER_CLUSTER": {
          "MAIN_DOOR": 0,
          "LEFT_FRONT_DOOR": 0,
          "RIGHT_BACK_DOOR": 0,
          "LEFT_BACK_DOOR": 0,
          "SEAT_BELT_WARNING": int(frame % 1300 == 900),
          "SEAT_BELT_WARNING2": 0,
          "LEFT_SIGNAL": frame % 300 < 70,
          "RIGHT_SIGNAL": frame % 310 < 70,
        },
        "PCM_BUTTONS": {
          "ACC_MAIN": frame % 500 == 1,
          "SET_MINUS": 4 <= frame % 500 < 10,
          "RES_PLUS": 80 <= frame % 500 < 250,
          "CANCEL": frame % 700 == 550,
        },
        "BUTTONS": {"LKC_BTN": frame % 300 < 5, "DISTANCE_BTN": frame % 100 < 5, "UI_SPEED": 50.0},
        "BRAKE": {"BRAKE_ENGAGED": frame % 800 == 750, "BRAKE_PRESSURE": 0.0},
        "WHEEL_SPEED": {"WHEELSPEED_F": frame % 1000 * 0.001},
        "TRANSMISSION": {"GEAR": rng.randrange(3)},
      }
      for controller, parser in ((old, old_parser), (new, new_parser)):
        controller.gas_raw_277 = 1000 if frame % 400 < 120 else 0
        controller.engine_rpm_raw_037 = frame % 3000
        for message, signals in updates.items():
          parser.vl[message].update(signals)
      self.assertEqual(normalize(old.update(old_parser)), normalize(new.update(new_parser)), f"button frame {frame}")
      self.assertEqual(getattr(old, BASE_REARM_FIELD), new.acc_rearm_edge)
      self.assertEqual(old.cruise_speed, new.cruise_speed)
      self.assertEqual(old.is_cruise_latch, new.is_cruise_latch)
      self.assertEqual(old.lkas_latch, new.lkas_latch)

  def test_interface_parameters_and_raw_observer(self):
    before = ast.parse(original_source("interface"))
    after = ast.parse((ROOT / "selfdrive/car/dnga/interface.py").read_text())
    classes = [
      next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "CarInterface") for tree in (before, after)
    ]
    methods = [{n.name: n for n in cls.body if isinstance(n, ast.FunctionDef)} for cls in classes]
    self.assertEqual(ast.dump(methods[0]["get_params"]), ast.dump(methods[1]["get_params"]))
    original_observer = methods[0]["update"].body[1:3]
    extracted_observer = methods[1]["_update_raw_can"].body[1:]
    self.assertEqual([ast.dump(n) for n in original_observer], [ast.dump(n) for n in extracted_observer])
    functions = []
    for tree in (before, after):
      ns = {}
      body = [n for n in tree.body if isinstance(n, ast.FunctionDef)]
      exec(compile(ast.Module(body=body, type_ignores=[]), "interface_decoders", "exec"), ns)
      functions.append(ns)
    for raw in range(65536):
      self.assertEqual(functions[0]["decode_engine_rpm_037"](raw), functions[1]["decode_engine_rpm_037"](raw))
    rng = random.Random(271)
    for _ in range(4000):
      for addr, name in ((0x271, "decode_stock_acc_brake_271"), (0x273, "decode_stock_acc_cmd_273")):
        dat = bytearray(rng.getrandbits(8) for _ in range(rng.choice([0, 7, 8, 9])))
        if len(dat) == 8:
          dat[-1] = (addr + len(dat) + 2 + sum(dat[:-1])) & 255
        self.assertEqual(functions[0][name](dat), functions[1][name](dat))

  def test_fingerprints_and_safety_configuration(self):
    for name in ("CAR", "FINGERPRINTS", "ECU_FINGERPRINT", "DBC", "NOT_CAN_CONTROLLED", "ACC_CAR", "SNG_CAR"):
      if name == "CAR":
        self.assertEqual(BEFORE.values.CAR.YARISCROSSHEV, AFTER.values.CAR.YARISCROSSHEV)
      else:
        self.assertEqual(getattr(BEFORE.values, name), getattr(AFTER.values, name))
    for path in ("opendbc/dnga_hev.dbc", "selfdrive/car/dnga/dnga_hev.dbc"):
      self.assertEqual(subprocess.check_output(["git", "show", f"{BASE}:{path}"], cwd=ROOT), (ROOT / path).read_bytes())


if __name__ == "__main__":
  result = unittest.main(exit=False)
  print(
    f"Compared {TestControllerBehavior.frames:,} control frames and {TestControllerBehavior.messages:,} CAN messages."
  )
  print("Brake states:", {hex(k): v for k, v in sorted(TestControllerBehavior.states.items())})
  print("Fault paths:", dict(TestControllerBehavior.faults))
  sys.exit(not result.result.wasSuccessful())
