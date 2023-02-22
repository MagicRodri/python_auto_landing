"""
This script is used to land the drone on a marker using the Aruco marker pose estimation.
"""
import logging
import math
import time
from pathlib import Path

import numpy as np
from pymavlink import mavutil

from aruco_tracker import ArucoSingleTracker
from utils import (
    camera_to_uav,
    change_mode,
    check_angle_descend,
    get_location_metres,
    marker_position_to_angle,
    request_message_interval,
    uav_to_ne,
)

logging.basicConfig(level=logging.INFO)
BASE_DIR = Path(__file__).resolve().parent

# Connect to the vehicle
vehicle = mavutil.mavlink_connection('udp:127.0.0.1:14551')

# Wait for the heartbeat message to confirm the connection
vehicle.wait_heartbeat()
logging.info("Heartbeat from system (system %u component %u)" %
             (vehicle.target_system, vehicle.target_component))

logging.info("Arming motors")
# Copter should arm in GUIDED mode
change_mode(vehicle, "GUIDED")

# Arming
vehicle.arducopter_arm()
vehicle.motors_armed_wait()

# Confirm vehicle armed before attempting to take off

# Take off to 10m
altitude = 0.5
vehicle.mav.command_long_send(vehicle.target_system, vehicle.target_component,
                              mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0,
                              0, 0, 0, altitude)

# Wait until the vehicle reaches the desired altitude
while True:
    pos_msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    current_altitude = pos_msg.to_dict().get("relative_alt") / 1000.0
    logging.info("Altitude:%s" % current_altitude)
    if current_altitude >= altitude * 0.95:
        logging.info("Reached target altitude")
        break

# Parameters
rad_2_deg = 180.0 / math.pi
deg_2_rad = 1.0 / rad_2_deg

# Define marker Tag
id_to_find = 1
marker_size = 10  #- [cm]
freq_send = 1  #- Hz

land_alt_cm = 50.0
angle_descend = 20 * deg_2_rad
land_speed_cms = 30.0

# Get the camera calibration path
calib_path = BASE_DIR / 'calibration_data/'
camera_matrix = np.loadtxt(calib_path / 'cameraMatrix.txt', delimiter=',')
camera_distortion = np.loadtxt(calib_path / 'cameraDistortion.txt',
                               delimiter=',')

aruco_tracker = ArucoSingleTracker(id_to_find=id_to_find,
                                   marker_size=marker_size,
                                   show_video=True,
                                   video_device=4,
                                   camera_matrix=camera_matrix,
                                   camera_distortion=camera_distortion)

time_0 = time.time()
while True:

    marker_found, x_cm, y_cm, z_cm = aruco_tracker.track(loop=False)
    if marker_found:
        x_cm, y_cm = camera_to_uav(x_cm, y_cm)
        uav_location = vehicle.recv_match(type='GLOBAL_POSITION_INT',
                                          blocking=True).to_dict()

        # If high altitude, use baro rather than visual
        if uav_location['relative_alt'] >= 5000.0:
            z_cm = uav_location['relative_alt'] / 10.0

        angle_x, angle_y = marker_position_to_angle(x_cm, y_cm, z_cm)

        if time.time() >= time_0 + 1.0 / freq_send:
            time_0 = time.time()

            logging.info(" ")
            logging.info("Altitude = %.0fcm" % z_cm)
            logging.info(
                "Marker found x = %5.0f cm  y = %5.0f cm -> angle_x = %5f  angle_y = %5f"
                % (x_cm, y_cm, angle_x * rad_2_deg, angle_y * rad_2_deg))

            vehicle_attitude = vehicle.recv_match(type='ATTITUDE',
                                                  blocking=True).to_dict()
            north, east = uav_to_ne(x_cm, y_cm, vehicle_attitude['yaw'])
            logging.info(
                "Marker N = %5.0f cm   E = %5.0f cm   Yaw = %.0f deg" %
                (north, east, vehicle_attitude['yaw'] * rad_2_deg))

            marker_lat, marker_lon = get_location_metres(
                uav_location, north * 0.01, east * 0.01)

            if check_angle_descend(angle_x, angle_y, angle_descend):
                # If angle is good enough, descend
                logging.info("Low error: descending")
                location_marker = (marker_lat, marker_lon,
                                   uav_location['relative_alt'] / 1000.0 -
                                   (land_speed_cms * 0.01 / freq_send))
            else:
                # Else get closer at the current altitude
                location_marker = (marker_lat, marker_lon,
                                   uav_location['relative_alt'] / 1000.0)

            vehicle.mav.mission_item_send(
                0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2, 0, 0, 0, 0, 0,
                *location_marker)
            logging.info(
                "UAV Location    Lat = %.7f  Lon = %.7f" %
                (uav_location['lat'] / 1e7, uav_location['lon'] / 1e7))
            logging.info("Commanding to   Lat = %.7f  Lon = %.7f" %
                         (location_marker[0], location_marker[1]))

        # Command to land
        if z_cm <= land_alt_cm:
            if vehicle.flightmode == "GUIDED":
                logging.info(" -->>COMMANDING TO LAND<<")
                change_mode(vehicle, 'LAND')

# vehicle.arducopter_disarm()
# vehicle.motors_diarmed_wait()