import logging
import time

from pymavlink import mavutil

from utils import change_mode

logging.basicConfig(level=logging.INFO)

# Connect to the flight controller
master = mavutil.mavlink_connection(device="127.0.0.1:14551")
boot_time = time.time()
# Wait for the heartbeat message to confirm the connection
master.wait_heartbeat()
logging.info("Heartbeat from system (system %u component %u)" %
             (master.target_system, master.target_component))

master.set_mode("GUIDED_NOGPS")

# Arm the motors
master.arducopter_arm()
# Wait for the arming to complete
master.motors_armed_wait()
logging.info("Armed!")

master.mav.command_long_send(master.target_system, master.target_component,
                             mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0,
                             0, 0, 0, 0.5)