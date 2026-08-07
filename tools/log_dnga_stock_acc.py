#!/usr/bin/env python3
"""Live read-only stock ACC transition logger with TX contamination checks."""

from dnga_logger_common import run_logger


if __name__ == "__main__":
  run_logger(
    "stock",
    "Log stock DNGA ACC braking, regen, acceleration, 0x275, and raw hybrid candidates.",
  )

