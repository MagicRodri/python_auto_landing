# Python auto landing system

## Description

This is a simple auto landing system for the ardupilot autopilot. It is uses opencv and python to detect the landing pad, in this case, an aruco board. It then uses the aruco board to calculate the distance and angle to the landing pad. Those data are then fed to the autopilot via mavlink. This enables both precision landing and loiter.

## Requirements

## Test results

Tests were conducted using a little quadcopter with a orange pi 3lts as companion computer running ubuntu 22.04 and mini-pix as flight controller running arducopter 4.3 .The companion computer was connected to the autopilot via usb for simplicity. The script runs on computer's start as service.

- Precision landing  
 ![Precision landing](data/landing.gif)
<!-- - Loiter -->
