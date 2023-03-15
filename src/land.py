import logging
from utils import change_mode
from pymavlink import mavutil

logging.basicConfig(level=logging.INFO)


# Connect to the flight controller
master = mavutil.mavlink_connection('udp:127.0.0.1:14551')

# Wait for the heartbeat message to confirm the connection
master.wait_heartbeat()
logging.info("Heartbeat from system (system %u component %u)" %
             (master.target_system, master.target_component))

change_mode(master,'LAND')
