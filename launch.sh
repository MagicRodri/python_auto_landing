#!/bin/bash
if [ -e /dev/ttyUSB0 ]; then
    mavproxy.py --master=/dev/ttyUSB0 --out=udp:127.0.0.1:14550  --out=udp:127.0.0.1:14551 --daemon &
    cd /home/rodrigue/dev/python_auto_landing/
    source venv/bin/activate
    python3 src/precise_landing2.py
elif [ -e /dev/ttyUSB1 ]; then
    mavproxy.py --master=/dev/ttyUSB1 --out=udp:127.0.0.1:14550  --out=udp:127.0.0.1:14551 --daemon &
    cd /home/rodrigue/dev/python_auto_landing/
    source venv/bin/activate
    python3 src/precise_landing2.py
else
    echo "No flight controller connected"
fi
