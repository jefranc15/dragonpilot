#!/bin/sh

BASE=/data/openpilot
LOGGER="$BASE/dnga_mapping_logger_v2.py"
PIDFILE="$BASE/dnga_mapping_logger_v2.pid"
STATUSFILE="$BASE/dnga_mapping_logger_v2_status.txt"
COMMANDS="$BASE/dnga_mapping_v2_commands.csv"

is_running() {
  if [ ! -f "$PIDFILE" ]; then
    return 1
  fi

  PID=$(cat "$PIDFILE" 2>/dev/null)
  [ -n "$PID" ] && kill -0 "$PID" 2>/dev/null
}

start_logger() {
  if [ ! -f "$LOGGER" ]; then
    echo "Logger not found: $LOGGER"
    exit 1
  fi

  if is_running; then
    echo "Logger already running. PID: $(cat "$PIDFILE")"
    exit 0
  fi

  rm -f "$PIDFILE"
  : > "$COMMANDS"

  cd "$BASE" || exit 1

  nohup env \
    PYTHONUNBUFFERED=1 \
    PYTHONPATH=/data/openpilot:/data/openpilot/cereal \
    python "$LOGGER" \
    > "$STATUSFILE" 2>&1 &

  PID=$!
  echo "$PID" > "$PIDFILE"
  sleep 3

  if kill -0 "$PID" 2>/dev/null; then
    echo "DNGA mapping logger v2 started."
    echo "PID: $PID"
    echo "SSH can now be closed safely."
    echo
    tail -n 15 "$STATUSFILE"
  else
    echo "Logger failed to start."
    cat "$STATUSFILE"
    rm -f "$PIDFILE"
    exit 1
  fi
}

stop_logger() {
  if ! is_running; then
    echo "Logger is not running."
    rm -f "$PIDFILE"
    exit 0
  fi

  PID=$(cat "$PIDFILE")
  echo "Stopping logger PID $PID..."
  kill -INT "$PID" 2>/dev/null

  COUNT=0
  while kill -0 "$PID" 2>/dev/null; do
    sleep 1
    COUNT=$((COUNT + 1))

    if [ "$COUNT" -ge 15 ]; then
      echo "INT timeout; sending TERM."
      kill -TERM "$PID" 2>/dev/null
      break
    fi
  done

  rm -f "$PIDFILE"
  echo
  tail -n 25 "$STATUSFILE" 2>/dev/null
}

status_logger() {
  if is_running; then
    echo "Logger running. PID: $(cat "$PIDFILE")"
  else
    echo "Logger not running."
  fi

  tail -n 15 "$STATUSFILE" 2>/dev/null
}

capture_event() {
  shift
  SECONDS="$1"
  shift
  LABEL="$*"

  if [ -z "$SECONDS" ] || [ -z "$LABEL" ]; then
    echo "Usage: sh $0 capture SECONDS LABEL"
    exit 1
  fi

  if ! is_running; then
    echo "Start the logger first."
    exit 1
  fi

  echo "capture,$SECONDS,$LABEL" >> "$COMMANDS"
  echo "Capture started: $LABEL"
  echo "Duration after command: $SECONDS seconds"
  echo "It also saves approximately 6 seconds before the command."
}

acc_main_capture() {
  if ! is_running; then
    echo "Start the logger first."
    exit 1
  fi

  echo "capture,20,ACC_MAIN" >> "$COMMANDS"
  echo "ACC_MAIN capture started for 20 seconds."
  echo
  echo "While PARKED:"
  echo "  1. Wait 2 seconds."
  echo "  2. Press and release ACC MAIN once."
  echo "  3. Wait 3 seconds."
  echo "  4. Press and release it again."
  echo "  5. Wait 3 seconds."
  echo "  6. Press and release it a third time."
  echo
  echo "Do not press SET, RES, CANCEL, DISTANCE, or LKA."
}

show_logs() {
  echo "Summary logs:"
  ls -lht "$BASE"/dnga_mapping_v2_summary_*.csv.gz \
    2>/dev/null | head -n 10
  echo
  echo "Raw event CAN logs:"
  ls -lht "$BASE"/dnga_mapping_v2_raw_*.csv.gz \
    2>/dev/null | head -n 10
  echo
  echo "Event indexes:"
  ls -lht "$BASE"/dnga_mapping_v2_events_*.csv \
    2>/dev/null | head -n 10
}

case "$1" in
  start)
    start_logger
    ;;
  stop)
    stop_logger
    ;;
  restart)
    stop_logger
    sleep 1
    start_logger
    ;;
  status)
    status_logger
    ;;
  tail)
    tail -n 50 "$STATUSFILE"
    ;;
  capture)
    capture_event "$@"
    ;;
  accmain)
    acc_main_capture
    ;;
  logs)
    show_logs
    ;;
  *)
    echo "Usage:"
    echo "  sh $0 start"
    echo "  sh $0 stop"
    echo "  sh $0 restart"
    echo "  sh $0 status"
    echo "  sh $0 tail"
    echo "  sh $0 capture SECONDS LABEL"
    echo "  sh $0 accmain"
    echo "  sh $0 logs"
    exit 1
    ;;
esac
