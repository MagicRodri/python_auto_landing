import logging
import math
from pathlib import Path
import time

import time
import numpy as np
from aruco_tracker import ArucoSingleTracker
from pymavlink import mavutil
from utils import play_buzzer_tune,send_altitude,marker_position_to_angle,send_landing_target

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
            play_buzzer_tune(master,"T200O2L1A#") # constantly play tune when marker found
            x_cm, y_cm = camera_to_target(x_cm, y_cm)
            z_cm = round(z_cm,2)
            logging.info(f"Marker found at ({x_cm}, {y_cm}, {z_cm})")
            angle_x, angle_y = marker_position_to_angle(x_cm, y_cm, z_cm)
            logging.info(f"Marker position in uav coordinates: ({angle_x}rad, {angle_y}rad)")
        
            if time.time() >= time_0 + 1.0/freq_send:
                time_0 = time.time()
                distance_m = math.sqrt(x_cm*x_cm + y_cm*y_cm + z_cm*z_cm) * 0.01 # 3D distance, maybe altitude?
                send_landing_target(master,
                                    distance=distance_m,
                                    x_angle=angle_x,
                                    y_angle=angle_y,
                                    time_usec=int(time.time()*1e6))
                send_altitude(master, int(z_cm))
                time.sleep(0.2)

if __name__ == "__main__":
    main()