"""DNGA camera ACC 0x273 observability decoder (no CAN actuation).

R1: A checksum-valid OEM inactive 0x273 must be recognized immediately.
The extra constant bits used during engaged ACC cannot safely be assumed
present in OEM disabled/ready payloads. Never infer engagement from
"SET_ME_1_2" or "SET_ME_1"; require the camera's enabled bit.
"""


def dnga_rx_checksum_valid(addr, dat):
  """Validate the OEM additive checksum used for 0x271 and 0x273."""
  try:
    dat = bytes(dat)
  except (TypeError, ValueError):
    return False
  if len(dat) != 8:
    return False
  expected = (addr + len(dat[:-1]) + 1 + 2 + sum(dat[:-1])) & 0xFF
  return dat[-1] == expected


def decode_stock_acc_cmd_273(dat):
  """Decode (enabled, lead, accel, decel, ACC_CMD_kph) for OEM 0x273.

  All checksum-valid *inactive* camera requests are treated as disabled,
  irrespective of constants found in engaged samples. No potentially
  untrusted lead/mode/desired-speed fields from an inactive message are
  passed to the stock longitudinal monitor.

  An enabled request still requires the observed engaged constants.
  A checksum-invalid or malformed frame is rejected (None), so the caller
  can invalidate previously cached cruise intent immediately.
  """
  try:
    dat = bytes(dat)
  except (TypeError, ValueError):
    return None
  if not dnga_rx_checksum_valid(0x273, dat):
    return None

  enabled = bool(dat[1] & 0x20)
  if not enabled:
    return False, False, False, False, 0.0

  # Strict known-engaged signature. An unrecognized engaged payload is
  # NOT authorized merely because bit 5 in byte 1 happens to be high.
  if not (dat[1] & 0x02) or not (dat[6] & 0x40):
    return None

  lead = bool(dat[1] & 0x08)
  is_decel = bool(dat[4] & 0x20)
  is_accel = bool(dat[4] & 0x40)
  acc_cmd_kph = ((dat[2] << 8) | dat[3]) * 0.01
  return True, lead, is_accel, is_decel, acc_cmd_kph
