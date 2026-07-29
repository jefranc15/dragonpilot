#!/bin/sh
set -eu

cd /data/openpilot

chmod +x dnga_solo_logger.py
chmod +x dnga_solo_logger_ctl.sh

PYTHONPATH=/data/openpilot:/data/openpilot/cereal \
python -m py_compile dnga_solo_logger.py

echo
echo "DNGA solo logger installed."
echo "No vehicle code was modified."
echo
echo "Start once while parked:"
echo "  sh /data/openpilot/dnga_solo_logger_ctl.sh start"
echo
echo "Watch the timed button phases:"
echo "  sh /data/openpilot/dnga_solo_logger_ctl.sh watch"
echo
echo "Stop after the drive:"
echo "  sh /data/openpilot/dnga_solo_logger_ctl.sh stop"
