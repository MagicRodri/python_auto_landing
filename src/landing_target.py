import logging
import math
from pathlib import Path
import time
from typing import Union
import os

import numpy as np
from aruco_tracker import ArucoSingleTracker
from pymavlink import mavutil
from utils import change_mode


def send_altitude(master: Union[mavutil.mavfile, mavutil.mavudp,
                                      mavutil.mavtcp],distance:int):
    """
    Send altitude to FCU to enable altitude hold
    """
    sensor_id = 1
    orientation = 25
    covariance = 70
    master.mav.distance_sensor_send(0, 10, 1000, distance,
                                    mavutil.mavlink.MAV_DISTANCE_SENSOR_ULTRASOUND, sensor_id, orientation,
                                    covariance)
    # logging.info(f"Sending rangefinder altitude: {distance}")

def send_landing_target(master: Union[mavutil.mavfile,
                                           mavutil.mavudp,
                                           mavutil.mavtcp],
                        distance:float,
                        x_angle:float,
                        y_angle:float,
                        time_usec:int):
    x = 0.0
    y = 0.0
    z = 0.0
    msg = master.mav.landing_target_encode(
    time_usec, # Timestamp
    0, # Target num
    mavutil.mavlink.MAV_FRAME_BODY_NED, # Frame,not used?
    x_angle,
    y_angle,
    distance, # Distance
    0.2, # size_x
    0.2, # size_y
    # x, # X position in meters
    # y, # Y position in meters
    # z, # Z position in meters
    # (1,0,0,0), # quaternion
    # 2, # Target type: 2 = Fiducial marker
    # 1, # position valid
    )
    master.mav.send(msg)   
    # logging.info("sent landing target")

def marker_position_to_angle(x:float, y:float, z:float):
    """
    Converts marker position to angle
    """
    angle_x = round(math.atan2(x,z),2)
    angle_y = round(math.atan2(y,z),2)
    
    return (angle_x, angle_y)

def camera_to_target(x_cam:float, y_cam:float):
    x_target,y_target = round(x_cam,2),round(y_cam,2)
    return (x_target, y_target) # target position with respect to camera

def main():

    id_to_find = 1
    marker_size = 10  # [cm]
    freq_send       = 15 #- Hz

    # camera calibration path
    calib_path = Path('__file__').resolve().parent /'src' / 'calibration_data'
    camera_matrix = np.loadtxt(calib_path / 'cameraMatrix.txt',
                               delimiter=',')
    camera_distortion = np.loadtxt(calib_path / 'cameraDistortion.txt',
                                   delimiter=',')
    aruco_tracker = ArucoSingleTracker(id_to_find=id_to_find,
                                       marker_size=marker_size,
                                       video_device=1,
                                       show_video=False,
                                       camera_matrix=camera_matrix,
                                       camera_distortion=camera_distortion)
    time_0 = time.time()

    master = mavutil.mavlink_connection(device="127.0.0.1:14551")
    master.wait_heartbeat()
    while True:
        logging.basicConfig(level=logging.INFO)
        marker_found, x_cm, y_cm, z_cm = aruco_tracker.track(loop=False,verbose=False)
        if marker_found:
            x_cm, y_cm = camera_to_target(x_cm, y_cm)
            z_cm = round(z_cm,2)
            logging.info(f"Marker found at ({x_cm}, {y_cm}, {z_cm})")
            angle_x, angle_y = marker_position_to_angle(x_cm, y_cm, z_cm)
            logging.info(f"Marker position in uav coordinates: ({angle_x}rad, {angle_y}rad)")
        
            if time.time() >= time_0 + 1.0/freq_send:
                time_0 = time.time()
                send_landing_target(master,
                                    distance=z_cm*0.01,
                                    x_angle=angle_x,
                                    y_angle=angle_y,
                                    time_usec=int(time.time()*1e6))
                send_altitude(master, int(z_cm))
                time.sleep(0.2)

if __name__ == "__main__":
    main()