# Python auto landing system

## Description
This project presents an auto landing system for the Ardupilot autopilot, utilizing OpenCV and Python for robust landing pad detection and Aruco marker recognition. The system calculates accurate distance and angle measurements to the landing pad, providing essential data to the autopilot via MAVLink protocol, thereby enabling precise landing and loitering capabilities.

## Requirements
 - Ardupilot-compatible drone  
 - Camera module (compatible with OpenCV)
 - Companion computer (e.g.,Orange Pi, Raspberry Pi, Nvidia Jetson) for onboard image processing
 - Python 3.x with [required](requirements.txt) libraries
 - Ground control station software (Mission Planner, QGroundControl, etc.)

## Test results

Tests were conducted using a home made mini quadcopter with an orange pi 3lts as companion computer running ubuntu 22.04 and mini-pix as flight controller running arducopter 4.3. The companion computer was connected to the autopilot via usb for simplicity. The script runs on computer's start as a service.

- Precision landing  
 ![Precision landing](data/landing.gif)
<!-- - Loiter -->
