#!/bin/sh

BASE=/data/openpilot
LOGGER="$BASE/dnga_full_mapping_logger.py"
PIDFILE="$BASE/dnga_full_mapping_logger.pid"
STATUSFILE="$BASE/dnga_full_mapping_logger_status.txt"
COMMANDS="$BASE/dnga_full_mapping_commands.csv"

start_logger() {
  if [ ! -f "$LOGGER" ]; then
    echo "Logger not found: $LOGGER"
    exit 1
  fi

  if [ -f "$PIDFILE" ]; then
    OLD_PID=$(cat "$PIDFILE" 2>/dev/null)
    if [ -n "$OLD_PID" ] && kill -0 "$OLD_PID" 2>/dev/null; then
      echo "Logger already running. PID: $OLD_PID"
      exit 0
    fi
    rm -f "$PIDFILE"
  fi

  cd "$BASE" || exit 1
  : > "$COMMANDS"

  nohup env \
    PYTHONUNBUFFERED=1 \
    PYTHONPATH=/data/openpilot:/data/openpilot/cereal \
    python "$LOGGER" \
    > "$STATUSFILE" 2>&1 &

  PID=$!
  echo "$PID" > "$PIDFILE"
  sleep 3

  if kill -0 "$PID" 2>/dev/null; then
    echo "DNGA full mapping logger started."
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
  if [ ! -f "$PIDFILE" ]; then
    echo "No PID file found."
    exit 0
  fi

  PID=$(cat "$PIDFILE" 2>/dev/null)

  if [ -n "$PID" ] && kill -0 "$PID" 2>/dev/null; then
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
  else
    echo "Logger was not running."
  fi

  rm -f "$PIDFILE"
  echo
  tail -n 25 "$STATUSFILE" 2>/dev/null
}

status_logger() {
  if [ -f "$PIDFILE" ]; then
    PID=$(cat "$PIDFILE" 2>/dev/null)
    if [ -n "$PID" ] && kill -0 "$PID" 2>/dev/null; then
      echo "Logger running. PID: $PID"
      tail -n 12 "$STATUSFILE"
      exit 0
    fi
  fi

  echo "Logger not running."
  tail -n 12 "$STATUSFILE" 2>/dev/null
}

show_logs() {
  echo "Summary:"
  ls -lht "$BASE"/dnga_full_mapping_summary_*.csv.gz \
    2>/dev/null | head -n 10
  echo
  echo "Raw CAN:"
  ls -lht "$BASE"/dnga_full_mapping_raw_*.csv.gz \
    2>/dev/null | head -n 10
  echo
  echo "Bit changes:"
  ls -lht "$BASE"/dnga_full_mapping_changes_*.csv.gz \
    2>/dev/null | head -n 10
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

  echo "capture,$SECONDS,$LABEL" >> "$COMMANDS"
  echo "Capture requested for $SECONDS seconds: $LABEL"
  echo "The logger also includes about 8 seconds before this command."
}

mark_event() {
  shift
  LABEL="$*"

  if [ -z "$LABEL" ]; then
    echo "Usage: sh $0 mark LABEL"
    exit 1
  fi

  echo "mark,$LABEL" >> "$COMMANDS"
  echo "Marker saved: $LABEL"
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
  logs)
    show_logs
    ;;
  capture)
    capture_event "$@"
    ;;
  mark)
    mark_event "$@"
    ;;
  *)
    echo "Usage:"
    echo "  sh $0 start"
    echo "  sh $0 stop"
    echo "  sh $0 restart"
    echo "  sh $0 status"
    echo "  sh $0 tail"
    echo "  sh $0 logs"
    echo "  sh $0 capture SECONDS LABEL"
    echo "  sh $0 mark LABEL"
    exit 1
    ;;
esac
