DNGA SOLO LOGGER 3.0

PURPOSE
- Start once while parked.
- Perform the timed button sequence.
- Close SSH and drive normally without a passenger.
- Stop it after the drive.

SAFETY
- Do not touch or watch the phone while driving.
- Complete the 90-second button sequence while parked.
- This logger is passive and does not transmit CAN.

START
  sh /data/openpilot/dnga_solo_logger_ctl.sh start

WATCH PHASES WHILE PARKED
  sh /data/openpilot/dnga_solo_logger_ctl.sh watch

TIMED PHASES
  0-5 sec:   BASELINE, press nothing
  5-15 sec:  ACC_MAIN, press/release 3 times
  15-25 sec: SET_MINUS, press/release 3 times
  25-35 sec: RES_PLUS, press/release 3 times
  35-45 sec: CANCEL, press/release 3 times
  45-55 sec: DISTANCE, press/release 3 times
  55-65 sec: LKA, press/release 3 times
  65-75 sec: NO_BUTTON_BASELINE
  75-90 sec: PREPARE_TO_DRIVE
  90+ sec:   DRIVE

You can press Ctrl+C while watching. It only closes tail; the logger
continues in the background.

STOP AFTER THE DRIVE
  sh /data/openpilot/dnga_solo_logger_ctl.sh stop
  sh /data/openpilot/dnga_solo_logger_ctl.sh logs

UPLOAD ALL FOUR FILES WITH THE SAME TIMESTAMP
- dnga_solo_summary_*.csv.gz
- dnga_solo_selected_can_*.csv.gz
- dnga_solo_raw_events_*.csv.gz
- dnga_solo_events_*.csv
