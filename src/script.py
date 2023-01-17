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
    vehicle = dronekit.connect("/dev/ttyUSB0", baud=57600, wait_ready=True)
else:
    print("Invalid input")
    sys.exit()

# Wait for the vehicle to be ready
# while not vehicle.is_armable:
#     print(" Waiting for vehicle to initialise...")
#     time.sleep(1)

# Set the vehicle's mode to "GUIDED"
print("Vehicle mode: ", vehicle.mode)
# Take off to 10m
vehicle.arm(wait=True)
vehicle.send_mavlink()
# Disconnect from the vehicle
vehicle.close()
