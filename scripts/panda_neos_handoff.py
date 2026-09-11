#!/usr/bin/env python3
import fcntl
import time

import usb1


FOREIGN_VID = 0x3801
LOCAL_VID = 0xBBAA
PANDA_APP_PID = 0xDDCC
PANDA_BOOTSTUB_PID = 0xDDEE

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


def request_shared_bootstub():
  ctx = usb1.USBContext()
  try:
    for dev in ctx.getDeviceList(skip_on_error=True):
      if dev.getVendorID() != FOREIGN_VID or dev.getProductID() != PANDA_APP_PID:
        continue

      log("Foreign LG Panda found at 3801:ddcc")
      try:
        handle = dev.open()
        try:
          # 0xd1, value 1 requests the Panda softloader/bootstub.
          # The shared bootstub accepts both the NEOS 0x2001FFFC and
          # LG/F413 0x2003FFFC handoff words.
          handle.controlWrite(0x40, 0xD1, 1, 0, b"", timeout=2000)
          log("Shared bootstub request returned normally")
        except Exception as e:
          # A disconnect error is expected when the Panda resets.
          log("Shared bootstub request result: %r" % (e,))
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


def wait_for_local_takeover(timeout=5.0):
  deadline = time.monotonic() + timeout
  while time.monotonic() < deadline:
    if usb_present(LOCAL_VID, PANDA_BOOTSTUB_PID):
      log("Shared Dragonpilot bootstub detected at bbaa:ddee")
      return True
    if usb_present(LOCAL_VID, PANDA_APP_PID):
      log("Dragonpilot Panda application detected at bbaa:ddcc")
      return True
    time.sleep(0.10)

  log("No bbaa Panda appeared after handoff request")
  return False


def handoff_once():
  if not usb_present(FOREIGN_VID, PANDA_APP_PID):
    return False

  if not request_shared_bootstub():
    return False

  return wait_for_local_takeover()


def main():
  lock = open(LOCK_PATH, "w")
  try:
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
  except BlockingIOError:
    return

  log("Panda NEOS handoff service started")

  armed = True
  all_absent_since = None

  while True:
    foreign_app = usb_present(FOREIGN_VID, PANDA_APP_PID)

    any_panda = (
      foreign_app or
      usb_present(FOREIGN_VID, PANDA_BOOTSTUB_PID) or
      usb_present(LOCAL_VID, PANDA_APP_PID) or
      usb_present(LOCAL_VID, PANDA_BOOTSTUB_PID)
    )

    if foreign_app and armed:
      armed = False
      all_absent_since = None
      try:
        handoff_once()
      except Exception as e:
        log("handoff error: %r" % (e,))

    # A reset can make USB disappear briefly. Re-arm only after a real unplug:
    # every known Panda identity must be continuously absent for three seconds.
    if any_panda:
      all_absent_since = None
    else:
      if all_absent_since is None:
        all_absent_since = time.monotonic()
      elif (not armed) and ((time.monotonic() - all_absent_since) >= 3.0):
        armed = True
        log("Panda fully absent for 3s; handoff service re-armed")

    time.sleep(0.50)


if __name__ == "__main__":
  main()
