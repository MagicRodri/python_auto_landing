# Import the necessary modules
import sys
import time

import dronekit

# Prompt the user to choose a connection type
connection_type = input("Enter 's' to connect to SITL or 'h' to connect to APM 2.6: ")

# Connect to the SITL or APM 2.6 based on the user's input
if connection_type == "s":
    # Connect to the SITL simulation on 127.0.0.1(localhost)
    vehicle = dronekit.connect("udp:127.0.0.1:14551", wait_ready=True)
elif connection_type == "h":
    # Connect to the APM 2.6 flight controller
    vehicle = dronekit.connect("/dev/ttyACM0", baud=115200, wait_ready=True)
else:
    print("Invalid input")
    sys.exit()

# Wait for the vehicle to be ready
while not vehicle.is_armable:
    print(" Waiting for vehicle to initialise...")
    time.sleep(1)

# Set the vehicle's mode to "GUIDED"
vehicle.mode = dronekit.VehicleMode("GUIDED")
vehicle.armed = True

# Wait for the mode to change
while vehicle.mode.name != "GUIDED":
    time.sleep(1)

# Take off to 10m
altitude = 10
vehicle.simple_takeoff(altitude)

# Wait until the vehicle reaches the desired altitude
while True:
    print("Altitude: ", vehicle.location.global_relative_frame.alt)
    if vehicle.location.global_relative_frame.alt >= altitude * 0.95:
        print("Reached target altitude")
        break
    time.sleep(1)

# Hover at the desired altitude for 10 seconds
counter = 0
while vehicle.mode.name == "GUIDED":
    print("Altitude: ", vehicle.location.global_relative_frame.alt)
    counter += 1
    time.sleep(1)
    if counter >= 10:
        break

# Land the vehicle
vehicle.mode = dronekit.VehicleMode("LAND")
print("Landing")

# Wait for the vehicle to land
while vehicle.armed:
    print("Waiting for vehicle to land")
    time.sleep(1)

print("Vehicle has landed")

# Disconnect from the vehicle
vehicle.close()
