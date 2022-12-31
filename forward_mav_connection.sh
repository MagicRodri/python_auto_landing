#!/bin/bash

# forward mavlink connection from flight controller USB port to udp port 14550 and 14551
mavproxy.py --master=/dev/ttyUSB0 --out=udp:127.0.0.1:14550  --out=udp:127.0.0.1:14551