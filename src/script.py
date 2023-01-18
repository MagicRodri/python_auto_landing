# Import the necessary modules
import sys
import time

import dronekit

# Prompt the user to choose a connection type
connection_type = input(
    "Enter 's' to connect to SITL or 'h' to connect to APM 2.6: ")

# Connect to the SITL or APM 2.6 based on the user's input
if connection_type == "s":
    # Connect to the SITL simulation on 127.0.0.1(localhost)
    vehicle = dronekit.connect("udp:127.0.0.1:14551", wait_ready=True)
elif connection_type == "h":
    # Connect to the APM 2.6 flight controller
    vehicle = dronekit.connect("/dev/ttyUSB0", baud=57600, wait_ready=True)
else:
    print("Invalid input")
    sys.exit()
while not vehicle.is_armable:
    print(" Waiting for vehicle to initialise...")
    time.sleep(1)

print("Arming motors")
# Copter should arm in GUIDED mode
vehicle.mode = dronekit.VehicleMode("GUIDED")
vehicle.armed = True

# Confirm vehicle armed before attempting to take off
while not vehicle.armed:
    print(" Waiting for arming...")
    time.sleep(1)

# Take off to 10m
altitude = 10
vehicle.simple_takeoff(altitude)

# Wait until the vehicle reaches the desired altitude
while True:
    print("Altitude: ", vehicle.location.global_relative_frame)
    print("Attitude: ", vehicle.attitude)
    if vehicle.location.global_relative_frame.alt >= altitude * 0.95:
        print("Reached target altitude")
        break
    time.sleep(1)
