#!/usr/bin/env python3
import fcntl
import glob
import os
import time

import usb1


FOREIGN_VID = 0x3801
PANDA_APP_PID = 0xDDCC
STM32_DFU_VID = 0x0483
STM32_DFU_PID = 0xDF11

LOCK_PATH = "/data/panda_neos_handoff.lock"


def log(msg):
  print(time.strftime("%Y-%m-%d %H:%M:%S"), msg, flush=True)


def usb_present(vid, pid):
  ctx = usb1.USBContext()
  try:
    for dev in ctx.getDeviceList(skip_on_error=True):
      if dev.getVendorID() == vid and dev.getProductID() == pid:
        return True
  except Exception as e:
    log("USB scan error: %r" % (e,))
  finally:
    try:
      ctx.close()
    except Exception:
      pass
  return False


def request_stm32_dfu():
  ctx = usb1.USBContext()
  try:
    for dev in ctx.getDeviceList(skip_on_error=True):
      if dev.getVendorID() != FOREIGN_VID or dev.getProductID() != PANDA_APP_PID:
        continue

      log("Foreign LG Panda found at 3801:ddcc")
      try:
        handle = dev.open()
        # 0xd1, value 0 asks the debug LG Panda application to jump directly
        # to the STM32 ROM bootloader. USB normally drops immediately.
        try:
          handle.controlWrite(0x40, 0xD1, 0, 0, b"", timeout=2000)
          log("STM32 DFU request returned normally")
        except Exception as e:
          log("STM32 DFU request result: %r" % (e,))
        try:
          handle.close()
        except Exception:
          pass
        return True
      except Exception as e:
        log("Unable to open foreign Panda: %r" % (e,))
        return False
  finally:
    try:
      ctx.close()
    except Exception:
      pass

  return False


def wait_for_dfu(timeout):
  deadline = time.monotonic() + timeout
  while time.monotonic() < deadline:
    if usb_present(STM32_DFU_VID, STM32_DFU_PID):
      return True
    time.sleep(0.10)
  return False


def force_usb_host_awake():
  # NEOS on OnePlus can put msm-dwc3/xHCI into runtime low power immediately
  # after the LG application disconnects. Pin the relevant devices awake first.
  power_paths = []
  power_paths.extend(glob.glob("/sys/bus/platform/devices/*ssusb*/power/control"))
  power_paths.extend(glob.glob("/sys/bus/platform/devices/*dwc3*/power/control"))
  power_paths.extend(glob.glob("/sys/bus/platform/devices/xhci*/power/control"))

  for path in sorted(set(power_paths)):
    try:
      with open(path, "w") as f:
        f.write("on")
      log("USB runtime PM forced on: %s" % path)
    except Exception:
      pass

  driver = "/sys/bus/platform/drivers/xhci-hcd"
  unbind = os.path.join(driver, "unbind")
  bind = os.path.join(driver, "bind")

  if not (os.path.exists(unbind) and os.path.exists(bind)):
    log("xHCI rebind interface not available")
    return False

  controllers = []
  for path in glob.glob(os.path.join(driver, "*")):
    name = os.path.basename(path)
    if name in ("bind", "unbind", "uevent", "module", "new_id", "remove_id"):
      continue
    if os.path.islink(path):
      controllers.append(name)

  if not controllers:
    log("No bound xHCI controller found")
    return False

  ok = False
  for controller in controllers:
    try:
      log("Rebinding xHCI controller %s" % controller)
      with open(unbind, "w") as f:
        f.write(controller)
      time.sleep(0.35)
      with open(bind, "w") as f:
        f.write(controller)
      ok = True
    except Exception as e:
      log("xHCI rebind failed for %s: %r" % (controller, e))

  return ok


def handoff_once():
  if not usb_present(FOREIGN_VID, PANDA_APP_PID):
    return False

  if not request_stm32_dfu():
    return False

  # Give the STM32 ROM a chance to enumerate without disturbing the host.
  if wait_for_dfu(1.25):
    log("STM32 DFU detected at 0483:df11")
    return True

  log("DFU not visible; waking and rescanning OnePlus USB host")
  force_usb_host_awake()

  if wait_for_dfu(4.0):
    log("STM32 DFU detected after USB host rescan")
    return True

  log("DFU still not visible; leaving Panda untouched until it is replugged")
  return False


def main():
  lock = open(LOCK_PATH, "w")
  try:
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
  except BlockingIOError:
    return

  log("Panda NEOS handoff service started")

  armed = True
  while True:
    foreign = usb_present(FOREIGN_VID, PANDA_APP_PID)

    if foreign and armed:
      armed = False
      try:
        handoff_once()
      except Exception as e:
        log("handoff error: %r" % (e,))

    # Re-arm only after the foreign application has disappeared. This prevents
    # the old endless 3801 -> reset -> 3801 loop.
    if not foreign:
      armed = True

    time.sleep(0.50)


if __name__ == "__main__":
  main()
