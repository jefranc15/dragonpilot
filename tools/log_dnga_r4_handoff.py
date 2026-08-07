#!/usr/bin/env python3
"""Live read-only R4 brake/regen/propulsion handoff logger."""

from dnga_logger_common import run_logger


if __name__ == "__main__":
  run_logger(
    "handoff",
    "Log DNGA R4 0x275 feedback, companion torque/brake signals, commands, and openpilot state.",
  )

