#!/bin/bash
if [ -e /dev/ttyUSB0 ]; then
    mavproxy.py --master=/dev/ttyUSB0 --out=udp:172.20.10.2:14550  --out=udp:127.0.0.1:14551
elif [ -e /dev/ttyUSB1 ]; then
    mavproxy.py --master=/dev/ttyUSB1 --out=udp:172.20.10.2:14550  --out=udp:127.0.0.1:14551
elif [ -e /dev/ttyACM0 ]; then
    mavproxy.py --master=/dev/ttyACM0 --out=udp:192.168.1.248:14550  --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551
else
    echo "No flight controller connected"
fi
