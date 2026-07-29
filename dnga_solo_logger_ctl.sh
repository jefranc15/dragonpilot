#!/bin/sh

BASE=/data/openpilot
LOGGER="$BASE/dnga_solo_logger.py"
PIDFILE="$BASE/dnga_solo_logger.pid"
STATUSFILE="$BASE/dnga_solo_logger_status.txt"

is_running() {
  if [ ! -f "$PIDFILE" ]; then
    return 1
  fi

  PID=$(cat "$PIDFILE" 2>/dev/null)
  [ -n "$PID" ] && kill -0 "$PID" 2>/dev/null
}

stop_old_loggers() {
  for SCRIPT in \
    dnga_full_mapping_logger_ctl.sh \
    dnga_mapping_logger_v2_ctl.sh \
    dnga_stock_brake_logger_ctl.sh \
    dnga_v25e_logger_ctl.sh
  do
    if [ -f "$BASE/$SCRIPT" ]; then
      sh "$BASE/$SCRIPT" stop >/dev/null 2>&1 || true
    fi
  done
}

start_logger() {
  if [ ! -f "$LOGGER" ]; then
    echo "Logger not found: $LOGGER"
    exit 1
  fi

  if is_running; then
    echo "Solo logger already running. PID: $(cat "$PIDFILE")"
    exit 0
  fi

  stop_old_loggers
  rm -f "$PIDFILE"

  cd "$BASE" || exit 1

  nohup env \
    PYTHONUNBUFFERED=1 \
    PYTHONPATH=/data/openpilot:/data/openpilot/cereal \
    python "$LOGGER" \
    > "$STATUSFILE" 2>&1 &

  PID=$!
  echo "$PID" > "$PIDFILE"
  sleep 3

  if ! kill -0 "$PID" 2>/dev/null; then
    echo "Logger failed to start."
    cat "$STATUSFILE"
    rm -f "$PIDFILE"
    exit 1
  fi

  echo "DNGA solo logger started. PID: $PID"
  echo
  echo "Remain PARKED for the first 90 seconds."
  echo "Watch the phase with:"
  echo "  sh $BASE/dnga_solo_logger_ctl.sh watch"
  echo
  echo "During each named button phase, press/release that button"
  echo "three times. Do not press other buttons in that phase."
  echo
  echo "After the phase becomes DRIVE, close SSH and drive normally."
  echo
  tail -n 12 "$STATUSFILE"
}

stop_logger() {
  if ! is_running; then
    echo "Solo logger is not running."
    rm -f "$PIDFILE"
    exit 0
  fi

  PID=$(cat "$PIDFILE")
  echo "Stopping solo logger PID $PID..."
  kill -INT "$PID" 2>/dev/null

  COUNT=0
  while kill -0 "$PID" 2>/dev/null; do
    sleep 1
    COUNT=$((COUNT + 1))

    if [ "$COUNT" -ge 20 ]; then
      echo "INT timeout; sending TERM."
      kill -TERM "$PID" 2>/dev/null
      break
    fi
  done

  rm -f "$PIDFILE"
  sync
  echo
  tail -n 30 "$STATUSFILE" 2>/dev/null
}

status_logger() {
  if is_running; then
    echo "Solo logger running. PID: $(cat "$PIDFILE")"
  else
    echo "Solo logger not running."
  fi
  tail -n 15 "$STATUSFILE" 2>/dev/null
}

watch_logger() {
  echo "Press Ctrl+C to stop watching. The logger will keep running."
  tail -f "$STATUSFILE"
}

show_logs() {
  echo "Summary:"
  ls -lht "$BASE"/dnga_solo_summary_*.csv.gz \
    2>/dev/null | head -n 10
  echo
  echo "Selected CAN:"
  ls -lht "$BASE"/dnga_solo_selected_can_*.csv.gz \
    2>/dev/null | head -n 10
  echo
  echo "Raw event CAN:"
  ls -lht "$BASE"/dnga_solo_raw_events_*.csv.gz \
    2>/dev/null | head -n 10
  echo
  echo "Event indexes:"
  ls -lht "$BASE"/dnga_solo_events_*.csv \
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
  watch)
    watch_logger
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
    echo "  sh $0 watch"
    echo "  sh $0 logs"
    exit 1
    ;;
esac
