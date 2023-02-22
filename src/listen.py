import logging

from pymavlink import mavutil

logging.basicConfig(level=logging.INFO)


def change_mode(mode: str = None):
    if mode is not None:
        mode_id = master.mode_mapping().get(mode)

        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)


# Connect to the flight controller
master = mavutil.mavlink_connection('udp:127.0.0.1:14551')

# Wait for the heartbeat message to confirm the connection
master.wait_heartbeat()
logging.info("Heartbeat from system (system %u component %u)" %
             (master.target_system, master.target_component))

# Arm the motors
master.arducopter_arm()
msg = master.recv_match(type='COMMAND_ACK', blocking=True)
print(msg.to_dict())

change_mode("GUIDED")
# Wait for the arming to complete
master.motors_armed_wait()
logging.info("Armed!")
