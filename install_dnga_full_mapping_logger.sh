#!/bin/sh
set -eu

cd /data/openpilot

chmod +x dnga_full_mapping_logger.py
chmod +x dnga_full_mapping_logger_ctl.sh

PYTHONPATH=/data/openpilot:/data/openpilot/cereal \
python -m py_compile dnga_full_mapping_logger.py

echo
echo "DNGA full mapping logger installed."
echo "No vehicle code was modified."
echo
echo "Start:"
echo "  sh /data/openpilot/dnga_full_mapping_logger_ctl.sh start"
echo
echo "Guaranteed parked button capture example:"
echo "  sh /data/openpilot/dnga_full_mapping_logger_ctl.sh capture 20 SET_MINUS"
echo
echo "Stop:"
echo "  sh /data/openpilot/dnga_full_mapping_logger_ctl.sh stop"
