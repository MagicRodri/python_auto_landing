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
                             0, 0, 0, 10)
time.sleep(1)
while True:
    # Local position msg
    # msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    # Nav controller output
    # msg = master.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)
    # Global int position
    pos_msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    att_msg = master.recv_match(type='ATTITUDE', blocking=True)
    print("location %s " % pos_msg.to_dict())
    print("Attitude %s" % att_msg.to_dict())
    time.sleep(1)