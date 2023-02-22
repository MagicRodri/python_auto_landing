import logging
import time

from pymavlink import mavutil

logging.basicConfig(level=logging.INFO)


def change_mode(mode: str = None):
    if mode is not None:
        mode_id = master.mode_mapping().get(mode)

        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)

        ack_msg = master.recv_match(type="COMMAND_ACK", blocking=True)
        if ack_msg is not None:
            ack_msg = ack_msg.to_dict()
            print(mavutil.mavlink.enums["MAV_RESULT"][
                ack_msg['result']].description)


# Connect to the flight controller
master = mavutil.mavlink_connection('udp:127.0.0.1:14551')

# Wait for the heartbeat message to confirm the connection
master.wait_heartbeat()
logging.info("Heartbeat from system (system %u component %u)" %
             (master.target_system, master.target_component))

change_mode("GUIDED")

# Arm the motors
master.arducopter_arm()
msg = master.recv_match(type='COMMAND_ACK', blocking=True)
print(msg.to_dict())

# Wait for the arming to complete
master.motors_armed_wait()
logging.info("Armed!")

master.mav.command_long_send(master.target_system, master.target_component,
                             mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0,
                             0, 0, 0, 0.5)
msg = master.recv_match(type='COMMAND_ACK', blocking=True)
print(msg.to_dict())

while True:
    # Current attitude
    msg = master.recv_match(type='GLOBAL_POSITION_INT',
                            blocking=True).to_dict()
    print(msg['relative_alt'])