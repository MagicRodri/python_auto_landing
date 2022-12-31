"""
This script is used to land the drone on a marker using the Aruco marker pose estimation.
"""
import argparse
import math
import time
from pathlib import Path

import numpy as np
from dronekit import LocationGlobalRelative, VehicleMode, connect

from aruco_tracker import ArucoSingleTracker
from utils import (
    camera_to_uav,
    check_angle_descend,
    get_location_metres,
    marker_position_to_angle,
    uav_to_ne,
)

BASE_DIR = Path(__file__).resolve().parent

# Parse connection argument
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default = '')
args = parser.parse_args()
        
# Connect to the vehicle
print('Connecting...')
vehicle = connect(args.connect, wait_ready=True)  

while not vehicle.is_armable:
    print(" Waiting for vehicle to initialise...")
    time.sleep(1)

print("Arming motors")
# Copter should arm in GUIDED mode
vehicle.mode    = VehicleMode("GUIDED")
vehicle.armed   = True

# Confirm vehicle armed before attempting to take off
while not vehicle.armed:
    print(" Waiting for arming...")
    time.sleep(1)

# Take off to 10m
altitude = 10
vehicle.simple_takeoff(altitude)

# Wait until the vehicle reaches the desired altitude
while True:
    print("Altitude: ", vehicle.location.global_relative_frame.alt)
    if vehicle.location.global_relative_frame.alt >= altitude * 0.95:
        print("Reached target altitude")
        break
    time.sleep(1)

# Parameters
rad_2_deg   = 180.0/math.pi
deg_2_rad   = 1.0/rad_2_deg 
    
# Define marker Tag
id_to_find      = 1
marker_size     = 10 #- [cm]
freq_send       = 1 #- Hz

land_alt_cm         = 50.0
angle_descend       = 20*deg_2_rad
land_speed_cms      = 30.0



# Get the camera calibration path
calib_path = BASE_DIR / 'calibration_data/'
camera_matrix   = np.loadtxt(calib_path / 'cameraMatrix.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path / 'cameraDistortion.txt', delimiter=',')   
                             
aruco_tracker       = ArucoSingleTracker(id_to_find=id_to_find, marker_size=marker_size, show_video=True, 
                camera_matrix=camera_matrix, camera_distortion=camera_distortion)
                
                
time_0 = time.time()

while True:                

    marker_found, x_cm, y_cm, z_cm = aruco_tracker.track(loop=False)
    if marker_found:
        x_cm, y_cm          = camera_to_uav(x_cm, y_cm)
        uav_location        = vehicle.location.global_relative_frame
        
        # If high altitude, use baro rather than visual
        if uav_location.alt >= 5.0:
            z_cm = uav_location.alt*100.0
            
        angle_x, angle_y    = marker_position_to_angle(x_cm, y_cm, z_cm)

        
        if time.time() >= time_0 + 1.0/freq_send:
            time_0 = time.time()
 
            print(" ")
            print("Altitude = %.0fcm"%z_cm)
            print("Marker found x = %5.0f cm  y = %5.0f cm -> angle_x = %5f  angle_y = %5f"%(x_cm, y_cm, angle_x*rad_2_deg, angle_y*rad_2_deg))
            
            north, east             = uav_to_ne(x_cm, y_cm, vehicle.attitude.yaw)
            print("Marker N = %5.0f cm   E = %5.0f cm   Yaw = %.0f deg"%(north, east, vehicle.attitude.yaw*rad_2_deg))
            
            marker_lat, marker_lon  = get_location_metres(uav_location, north*0.01, east*0.01)  
            # If angle is good, descend
            if check_angle_descend(angle_x, angle_y, angle_descend):
                print("Low error: descending")
                location_marker         = LocationGlobalRelative(marker_lat, marker_lon, uav_location.alt-(land_speed_cms*0.01/freq_send))
            else:
                location_marker         = LocationGlobalRelative(marker_lat, marker_lon, uav_location.alt)
                
            vehicle.simple_goto(location_marker)
            print("UAV Location    Lat = %.7f  Lon = %.7f"%(uav_location.lat, uav_location.lon))
            print("Commanding to   Lat = %.7f  Lon = %.7f"%(location_marker.lat, location_marker.lon))
            
        # Command to land
        if z_cm <= land_alt_cm:
            if vehicle.mode == "GUIDED":
                print (" -->>COMMANDING TO LAND<<")
                vehicle.mode = "LAND"
            
