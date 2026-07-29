DNGA V2.5J STANDALONE LOGGER
================================

Files to copy into /data/openpilot:

  dnga_v25j_logger.py
  dnga_v25j_logger_ctl.sh

The logger is read-only. It subscribes to cereal CAN and state services and
does not modify interface.py, carcontroller.py, dngacan.py, the DBC, or any
other openpilot file.

INSTALL
-------

Copy both files to /data/openpilot, then run:

  chmod +x /data/openpilot/dnga_v25j_logger_ctl.sh

START BEFORE THE DRIVE
----------------------

  sh /data/openpilot/dnga_v25j_logger_ctl.sh start

Wait until it says the logger started. You may then close SSH.

TEST TO CAPTURE
---------------

1. Start with the car parked and verify there are no warning-light flickers.
2. Engage openpilot normally.
3. Follow a moving lead car through at least three slowdown/recovery cycles.
4. Include gentle and moderate slowing, but do not intentionally create an
   emergency or stopped-lead test.
5. If you feel the on/off jerk, continue safely for a few seconds so the
   event window includes the recovery.
6. Drive for roughly 10-15 minutes if conditions allow.

STOP AFTER THE DRIVE
--------------------

  sh /data/openpilot/dnga_v25j_logger_ctl.sh stop

The stop command prints the exact three files to upload:

  dnga_v25j_summary_MMDD_HHMMSS.csv.gz
  dnga_v25j_selected_can_MMDD_HHMMSS.csv.gz
  dnga_v25j_raw_events_MMDD_HHMMSS.csv.gz

The summary file directly decodes V2.5J's 0x271 pump stage, proportional
deceleration command, ramp delta, target error, brake entries/releases,
rapid re-entry, chattering, positive-acceleration overlap, lead state, and
Panda/ignition state.

STATUS OR FILE NAMES
--------------------

  sh /data/openpilot/dnga_v25j_logger_ctl.sh status
  sh /data/openpilot/dnga_v25j_logger_ctl.sh files
