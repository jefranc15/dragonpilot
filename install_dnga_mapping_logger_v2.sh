#!/bin/sh
set -eu

cd /data/openpilot

chmod +x dnga_mapping_logger_v2.py
chmod +x dnga_mapping_logger_v2_ctl.sh

PYTHONPATH=/data/openpilot:/data/openpilot/cereal \
python -m py_compile dnga_mapping_logger_v2.py

echo
echo "DNGA mapping logger v2 installed."
echo "No vehicle code was modified."
echo
echo "Start:"
echo "  sh /data/openpilot/dnga_mapping_logger_v2_ctl.sh start"
echo
echo "ACC MAIN test:"
echo "  sh /data/openpilot/dnga_mapping_logger_v2_ctl.sh accmain"
echo
echo "Stop:"
echo "  sh /data/openpilot/dnga_mapping_logger_v2_ctl.sh stop"
