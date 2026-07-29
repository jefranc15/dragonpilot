#!/system/bin/sh

OPENPILOT_DIR="/data/openpilot"
LOGGER="$OPENPILOT_DIR/dnga_v25j_logger.py"
PIDFILE="$OPENPILOT_DIR/.dnga_v25j_logger.pid"

is_running() {
  if [ ! -f "$PIDFILE" ]; then
    return 1
  fi
  PID="$(cat "$PIDFILE" 2>/dev/null)"
  if [ -z "$PID" ]; then
    return 1
  fi
  kill -0 "$PID" 2>/dev/null
}

show_files() {
  if [ -f "$OPENPILOT_DIR/.dnga_v25j_last_stamp" ]; then
    STAMP="$(cat "$OPENPILOT_DIR/.dnga_v25j_last_stamp" 2>/dev/null)"
    echo "Files for the latest run:"
    echo "$OPENPILOT_DIR/dnga_v25j_summary_${STAMP}.csv.gz"
    echo "$OPENPILOT_DIR/dnga_v25j_selected_can_${STAMP}.csv.gz"
    echo "$OPENPILOT_DIR/dnga_v25j_raw_events_${STAMP}.csv.gz"
  else
    echo "No V2.5J logger run has been recorded yet."
  fi
}

case "$1" in
  start)
    if [ ! -f "$LOGGER" ]; then
      echo "Missing $LOGGER"
      exit 1
    fi
    if is_running; then
      echo "V2.5J logger is already running with PID $PID"
      exit 0
    fi

    rm -f "$PIDFILE"
    CONSOLE="$OPENPILOT_DIR/dnga_v25j_logger_console_$(date +%m%d_%H%M%S).txt"
    PYTHONPATH="$OPENPILOT_DIR:$OPENPILOT_DIR/cereal" \
      nohup python "$LOGGER" >"$CONSOLE" 2>&1 &
    PID=$!
    echo "$PID" > "$PIDFILE"
    sleep 2

    if kill -0 "$PID" 2>/dev/null; then
      echo "V2.5J logger started with PID $PID"
      echo "You can close SSH now."
      echo "Console log: $CONSOLE"
    else
      echo "Logger failed to start. Check:"
      echo "$CONSOLE"
      rm -f "$PIDFILE"
      exit 1
    fi
    ;;

  stop)
    if ! is_running; then
      echo "V2.5J logger is not running."
      rm -f "$PIDFILE"
      show_files
      exit 0
    fi

    echo "Stopping V2.5J logger PID $PID..."
    kill "$PID" 2>/dev/null
    COUNT=0
    while kill -0 "$PID" 2>/dev/null && [ "$COUNT" -lt 10 ]; do
      sleep 1
      COUNT=$((COUNT + 1))
    done

    if kill -0 "$PID" 2>/dev/null; then
      echo "Logger did not stop cleanly after 10 seconds."
      echo "Run: kill -9 $PID"
      exit 1
    fi

    rm -f "$PIDFILE"
    echo "V2.5J logger stopped and gzip files were closed."
    show_files
    ;;

  status)
    if is_running; then
      echo "V2.5J logger is running with PID $PID"
    else
      echo "V2.5J logger is not running."
    fi
    show_files
    ;;

  files)
    show_files
    ;;

  *)
    echo "Usage: sh $0 {start|stop|status|files}"
    exit 1
    ;;
esac
