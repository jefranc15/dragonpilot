#!/usr/bin/bash
size=$(du -sb .git/index 2>/dev/null|awk '{print $1}')
echo $size|grep -E '^[0-9]+$' >/dev/null || size=0
if [ $size -le 1024 ];then
    rm .git/index 2>/dev/null
    git reset
fi

export PASSIVE="0"

# OnePlus/NEOS: take over a Black Panda left running LG G8 firmware.
# The service only touches 3801:ddcc and hands STM32 DFU back to normal pandad.
if [ -f /EON ]; then
    nohup python -u ./scripts/panda_neos_handoff.py >> /data/panda_neos_handoff.log 2>&1 &
fi

exec ./launch_chffrplus.sh

