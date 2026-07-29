#!/bin/sh

BASE=/data/openpilot
LOGGER="$BASE/dnga_stock_brake_logger.py"
PIDFILE="$BASE/dnga_stock_brake_logger.pid"
STATUSFILE="$BASE/dnga_stock_brake_logger_status.txt"
MARKFILE="$BASE/dnga_stock_brake_markers.txt"

start_logger() {
  if [ ! -f "$LOGGER" ]; then
    echo "Logger not found: $LOGGER"
    exit 1
  fi

  if [ -f "$PIDFILE" ]; then
    OLD_PID=$(cat "$PIDFILE" 2>/dev/null)
    if [ -n "$OLD_PID" ] && kill -0 "$OLD_PID" 2>/dev/null; then
      echo "Logger is already running. PID: $OLD_PID"
      exit 0
    fi
    rm -f "$PIDFILE"
  fi

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
    echo "Stock/manual brake logger started."
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

  if [ -z "$PID" ]; then
    echo "Invalid PID file."
    rm -f "$PIDFILE"
    exit 1
  fi

  if kill -0 "$PID" 2>/dev/null; then
    echo "Stopping logger PID $PID..."
    kill -INT "$PID" 2>/dev/null

    COUNT=0
    while kill -0 "$PID" 2>/dev/null; do
      sleep 1
      COUNT=$((COUNT + 1))

      if [ "$COUNT" -ge 15 ]; then
        echo "INT did not stop it; sending TERM."
        kill -TERM "$PID" 2>/dev/null
        break
      fi
    done
  else
    echo "Logger was not running."
  fi

  rm -f "$PIDFILE"
  echo
  tail -n 20 "$STATUSFILE" 2>/dev/null
}

status_logger() {
  if [ -f "$PIDFILE" ]; then
    PID=$(cat "$PIDFILE" 2>/dev/null)
    if [ -n "$PID" ] && kill -0 "$PID" 2>/dev/null; then
      echo "Logger is running. PID: $PID"
      tail -n 12 "$STATUSFILE"
      exit 0
    fi
  fi

  echo "Logger is not running."
  tail -n 12 "$STATUSFILE" 2>/dev/null
}

show_logs() {
  echo "Summary logs:"
  ls -lht "$BASE"/dnga_stock_brake_summary_*.csv.gz \
    2>/dev/null | head -n 10
  echo
  echo "Raw event CAN logs:"
  ls -lht "$BASE"/dnga_stock_brake_raw_*.csv.gz \
    2>/dev/null | head -n 10
}

mark_event() {
  shift
  NOTE="$*"

  if [ -z "$NOTE" ]; then
    echo "Usage: sh $0 mark light_manual_brake"
    exit 1
  fi

  echo "$(date -Iseconds),$NOTE" >> "$MARKFILE"
  echo "Marker saved: $NOTE"
  echo "Only use markers while parked or through a passenger."
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
    tail -n 40 "$STATUSFILE"
    ;;
  logs)
    show_logs
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
    echo "  sh $0 mark DESCRIPTION"
    exit 1
    ;;
esac
