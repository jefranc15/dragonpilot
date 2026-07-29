#!/bin/sh
set -eu

BASE=/data/openpilot

echo "Copy these files into $BASE before running this installer:"
echo "  dnga_stock_brake_logger.py"
echo "  dnga_stock_brake_logger_ctl.sh"
echo

cd "$BASE"

chmod +x dnga_stock_brake_logger.py
chmod +x dnga_stock_brake_logger_ctl.sh

PYTHONPATH=/data/openpilot:/data/openpilot/cereal \
python -m py_compile dnga_stock_brake_logger.py

echo
echo "DNGA stock/manual brake logger installed."
echo "No car files were modified."
echo
echo "Start:"
echo "  sh /data/openpilot/dnga_stock_brake_logger_ctl.sh start"
echo
echo "Stop:"
echo "  sh /data/openpilot/dnga_stock_brake_logger_ctl.sh stop"
