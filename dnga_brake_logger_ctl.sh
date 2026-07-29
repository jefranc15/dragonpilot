#!/bin/sh

BASE=/data/openpilot
LOGGER="$BASE/dnga_brake_activation_logger.py"
PIDFILE="$BASE/dnga_brake_logger.pid"
STATUSFILE="$BASE/dnga_brake_logger_status.txt"

is_running() {
  [ -f "$PIDFILE" ] || return 1
  PID=$(cat "$PIDFILE" 2>/dev/null)
  [ -n "$PID" ] || return 1
  kill -0 "$PID" 2>/dev/null
}

start_logger() {
  if is_running; then
    echo "Logger is already running. PID: $PID"
    exit 0
  fi

  rm -f "$PIDFILE"

  if [ ! -f "$LOGGER" ]; then
    echo "Logger not found: $LOGGER"
    exit 1
  fi

  cd "$BASE" || exit 1

  nohup env PYTHONPATH="$BASE:$BASE/cereal" \
    python -u "$LOGGER" \
    > "$STATUSFILE" 2>&1 </dev/null &

  PID=$!
  echo "$PID" > "$PIDFILE"
  sleep 2

  if kill -0 "$PID" 2>/dev/null; then
    echo "Logger started. PID: $PID"
    echo "SSH can now be closed safely."
    echo "Status file: $STATUSFILE"
    tail -n 8 "$STATUSFILE"
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

  echo "Stopping logger PID $PID..."
  kill -TERM "$PID" 2>/dev/null

  COUNT=0
  while kill -0 "$PID" 2>/dev/null && [ "$COUNT" -lt 10 ]; do
    sleep 1
    COUNT=$((COUNT + 1))
  done

  if kill -0 "$PID" 2>/dev/null; then
    echo "Logger did not stop cleanly; forcing stop."
    kill -KILL "$PID" 2>/dev/null
  fi

  rm -f "$PIDFILE"
  echo "Logger stopped."
  tail -n 12 "$STATUSFILE" 2>/dev/null
  echo
  ls -lht "$BASE"/dnga_brake_activation_*.csv.gz 2>/dev/null | head -n 3
}

status_logger() {
  if is_running; then
    echo "Logger is running. PID: $PID"
  else
    echo "Logger is not running."
  fi
  tail -n 12 "$STATUSFILE" 2>/dev/null
}

case "${1:-status}" in
  start) start_logger ;;
  stop) stop_logger ;;
  restart) stop_logger; start_logger ;;
  status) status_logger ;;
  tail) tail -n 30 "$STATUSFILE" 2>/dev/null ;;
  logs) ls -lht "$BASE"/dnga_brake_activation_*.csv.gz 2>/dev/null | head -n 10 ;;
  *)
    echo "Usage: sh dnga_brake_logger_ctl.sh {start|stop|restart|status|tail|logs}"
    exit 1
    ;;
esac
