import logging

from pymavlink import mavutil

logging.basicConfig(level=logging.INFO)

# Connect to the flight controller
master = mavutil.mavlink_connection('udp:127.0.0.1:14551')

# Wait for the heartbeat message to confirm the connection
master.wait_heartbeat()
logging.info("Heartbeat from system (system %u component %u)" %
             (master.target_system, master.target_component))

print(master.mavlink10())
print(master.mavlink20())

# Arm the motors
master.arducopter_arm()
msg = master.recv_match(type='COMMAND_ACK', blocking=True)
print(msg.to_dict())

# Wait for the arming to complete
master.motors_armed_wait()
logging.info("Armed!")
