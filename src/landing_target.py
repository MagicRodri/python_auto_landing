import logging
import math
from pathlib import Path
import sys
import time

from pymavlink import mavutil

from board_detector import BoardDetector
from utils import play_buzzer_tune,send_altitude,marker_position_to_angle,send_landing_target

def camera_to_target(x_cam:float, y_cam:float):
    x_target,y_target = round(x_cam,2),round(y_cam,2)
    return (x_target, y_target) # target position with respect to camera

def main():
    logging.basicConfig(level=logging.INFO)
    BASE_DIR = Path(sys.argv[0]).resolve().parent
    BOARD_DATA_DIR = BASE_DIR / 'board_data'
    CALIBRATION_DIR = BASE_DIR / 'calibration_data'
    detector = BoardDetector(
        board_layout_path=BOARD_DATA_DIR / 'board_layout.yaml',
        calibration_path=CALIBRATION_DIR / 'calibration.yaml',
        detector_parameters_path=BOARD_DATA_DIR / 'detector_params.yaml',
        debug=False,
        video_device=1)
    
    X_OFFSET = 0.27 # m
    master = mavutil.mavlink_connection("/dev/ttyACM0")
    master.wait_heartbeat()
    
    while True:
        marker_found, coordinates,_  = detector.run(loop=False)
        if marker_found:
            x_m, y_m, z_m = coordinates
            x_m += X_OFFSET
            play_buzzer_tune(master,"T200O2L1A#") # continously play tune when marker found
            x_m, y_m = camera_to_target(x_m, y_m)
            z_m = round(z_m,2)
            logging.info(f"Marker found at ({x_m}, {y_m}, {z_m})")
            angle_x, angle_y = marker_position_to_angle(x_m, y_m, z_m)
            logging.info(f"Marker position in uav coordinates: ({angle_x}rad, {angle_y}rad)")
        
            distance_m = math.sqrt(x_m*x_m + y_m*y_m + z_m*z_m) # Distance to target
            send_landing_target(master,
                                distance=distance_m,
                                x_angle=angle_x,
                                y_angle=angle_y,
                                time_usec=int(time.time()*1e6))
            send_altitude(master, int(z_m * 100))
            # time.sleep(0.2)

if __name__ == "__main__":
    main()
