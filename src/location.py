import logging
import os
import time
from typing import Union

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


def request_message_interval(master: Union[mavutil.mavfile, mavutil.mavudp,
                                           mavutil.mavtcp], message_id: int,
                             frequency_hz: float) -> None:
    """
        Function to request mav message at given frequency
    """

    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                 0, message_id, 1e6 / frequency_hz, 0, 0, 0, 0,
                                 0)


# Connect to the flight controller
master = mavutil.mavlink_connection('udp:127.0.0.1:14551', baud=57600)

# Wait for the heartbeat message to confirm the connection
master.wait_heartbeat()
logging.info("Heartbeat from system (system %u component %u)" %
             (master.target_system, master.target_component))
# del os.environ['MAVLINK20']
# mavutil.set_dialect("ardupilotmega")
# master.mav.command_long_send(master.target_system, master.target_component,
#                              mavutil.mavlink.MAV_REQUEST_DATA_STREAM, 4, 4, 0)

request_message_interval(master,
                         mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 8)

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
while True:
    # Local position msg
    # msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    # Nav controller output
    # msg = master.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)
    # Global int position
    pos_msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    # att_msg = master.recv_match(type='ATTITUDE', blocking=True)
    alt = pos_msg.to_dict().get('relative_alt')
    print("Altitude: %s " % alt)
    # print("Attitude %s" % att_msg.to_dict())
    # if alt / 1000 > 9.8:
    #     break