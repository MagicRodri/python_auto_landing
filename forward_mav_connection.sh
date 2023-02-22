#!/bin/bash
if [ -e /dev/ttyUSB0 ]; then
    mavproxy.py --master=/dev/ttyUSB0 --out=udp:127.0.0.1:14550  --out=udp:127.0.0.1:14551
elif [ -e /dev/ttyUSB1 ]; then
    mavproxy.py --master=/dev/ttyUSB1 --out=udp:127.0.0.1:14550  --out=udp:127.0.0.1:14551
else
    echo "No flight controller connected"
fi
