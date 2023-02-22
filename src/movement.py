import logging
import time

from pymavlink import mavutil

from utils import change_mode

logging.basicConfig(level=logging.INFO)


def log_message(master=None, msg=None):
    if msg is None:
        msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg.to_dict())


# Connect to the flight controller
master = mavutil.mavlink_connection('udp:127.0.0.1:14551')

# Wait for the heartbeat message to confirm the connection
master.wait_heartbeat()
logging.info("Heartbeat from system (system %u component %u)" %
             (master.target_system, master.target_component))

# Arm the motors
master.arducopter_arm()
log_message(master)

# Wait for the arming to complete
master.motors_armed_wait()
logging.info("Armed!")

master.mav.command_long_send(master.target_system, master.target_component,
                             mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0,
                             0, 0, 0, 10)
time.sleep(5)
target_x = 10
# Send the vehicle to the given (x,y,z)
master.mav.send(
    mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        10, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b110111111000, target_x, 0, -15,
        0, 0, 0, 0, 0, 0, 1.57, 0))

# Sending vehicle to the given lat lon and alt
# master.mav.send(
#     mavutil.mavlink.MAVLink_set_position_target_global_int_message(
#         10, master.target_system, master.target_component,
#         mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 0b110111111000,
#         int(-35.3606274 * 10**7), int(149.1721487 * 10**7), 15, 0, 0, 0, 0, 0,
#         0, 1.57, 0))

# master.mav.mission_item_send(0, 0, 0,
#                              mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#                              mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2, 0, 0, 0,
#                              0, 0, location.lat, location.lon, alt)

while True:
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    print(msg.to_dict())
    if msg.to_dict()['x'] > target_x - 0.2:
        break
logging.info("Reached target position!")