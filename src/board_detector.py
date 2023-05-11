import logging
import math
from pathlib import Path
from typing import Tuple

import cv2
import numpy as np
import yaml

BASE_DIR = Path('__file__').resolve().parent / 'src'  # Run from root directory
BOARD_DATA_DIR = BASE_DIR / 'board_data'
CALIBRATION_DIR = BASE_DIR / 'calibration_data'


class BoardDetector:
    """Board detector class"""

    def __init__(self,
                 video_device: int = 0,
                 resolution: Tuple[int] = (640, 480),
                 debug: bool = False) -> None:
        self.video_device = video_device
        self.resolution = resolution
        self.debug = debug

        self._capture = cv2.VideoCapture(self.video_device)
        self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
        self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])

    def _rotation_matrix_to_euler_angles(self, R: np.array) -> np.array:
        """

        """

        def is_rotation_matrix(R):
            Rt = np.transpose(R)
            shouldBeIdentity = np.dot(Rt, R)
            I = np.identity(3, dtype=R.dtype)
            n = np.linalg.norm(I - shouldBeIdentity)
            return n < 1e-6

        assert (is_rotation_matrix(R))

        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])

    def _load_board_data(self, file_path: str | Path) -> Tuple[np.array]:
        with open(file_path, 'r') as file:
            data = yaml.load(file, Loader=yaml.SafeLoader)
            mm_px = data['mm_per_unit']
            corners = np.array(data['corners']).astype(np.float32) * mm_px
            ids = np.array(data['ids'])
            return corners, ids

    def _load_camera_parameters(self,
                                file_path: str | Path) -> Tuple[np.array]:
        with open(file_path, 'r') as file:
            data = yaml.load(file, Loader=yaml.SafeLoader)
            camera_matrix = np.array(data['camera_matrix'])
            distortion_coeffs = np.array(data['distortion_coeffs'])
            return camera_matrix, distortion_coeffs

    def _load_detector_parameters(
            self, file_path: str | Path) -> cv2.aruco_DetectorParameters:
        with open(file_path, 'r') as file:
            data = yaml.load(file, Loader=yaml.SafeLoader)
            params = cv2.aruco.DetectorParameters_create()
            for key, value in data.items():
                setattr(params, key, value)
            # params.adaptiveThreshWinSizeMin = data['adaptiveThreshWinSizeMin']
            # params.adaptiveThreshWinSizeMax = data['adaptiveThreshWinSizeMax']
            # params.adaptiveThreshWinSizeStep = data['adaptiveThreshWinSizeStep']
            # params.adaptiveThreshConstant = data['adaptiveThreshConstant']
            # params.minMarkerPerimeterRate = data['minMarkerPerimeterRate']
            # params.maxMarkerPerimeterRate = data['maxMarkerPerimeterRate']
            # params.polygonalApproxAccuracyRate = data[
            #     'polygonalApproxAccuracyRate']
            # params.minDistanceToBorder = data['minDistanceToBorder']
            # params.minMarkerDistanceRate = data['minMarkerDistanceRate']

            return params

    def run(self, loop: bool = True, display: bool = False) -> None | Tuple:
        # Load board data
        board_corners, board_ids = self._load_board_data(BOARD_DATA_DIR /
                                                         'board_layout.yaml')

        # Load camera parameters
        camera_matrix, distortion_coeffs = self._load_camera_parameters(
            CALIBRATION_DIR / 'calibration.yaml')

        # Define Aruco dictionary and board parameters
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)

        board = cv2.aruco.Board_create(board_corners, aruco_dict, board_ids)

        # Load detector parameters
        detector_params = self._load_detector_parameters(
            BOARD_DATA_DIR / 'detector_params.yaml')
        detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        if loop:
            logging.info('Detection started')

        while True:
            ret, frame = self._capture.read()
            if not ret:
                break

            # Convert the frame to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Detect the markers in the frame
            # returns corners, ids and rejected
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, board.dictionary, parameters=detector_params)

            # Draw the detected markers candidates
            image_markers = cv2.aruco.drawDetectedMarkers(
                frame.copy(), corners, ids)

            # Check if the detected markers correspond to the board's corners and ids
            valid = False
            if ids is not None and len(ids) > 0:
                if self.debug:
                    logging.info(f'Detected  {ids.size} markers')
                valid_ids = []
                valid_corners = []

                for i in range(len(ids)):
                    if ids[i][0] in board_ids:
                        valid_ids.append(ids[i][0])
                        valid_corners.append(corners[i])

                if len(valid_ids) > 0:
                    _, rvec, tvec = cv2.aruco.estimatePoseBoard(
                        np.array(valid_corners), np.array(valid_ids), board,
                        camera_matrix, distortion_coeffs, None, None)

                    if self.debug:
                        logging.info(f'rotation:{rvec}')
                        logging.info(f'translation:{tvec}')

                    if cv2.norm(tvec) > 0.00001:
                        cv2.aruco.drawAxis(image_markers, camera_matrix,
                                           distortion_coeffs, rvec, tvec, 0.4)
                        # Camera's coordinates in board's coordinate system (millimeters)
                        x = tvec[0, 0]
                        y = tvec[1, 0]
                        z = tvec[2, 0]
                        # the rotation matrix tag->camera
                        R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
                        R_tc = R_ct.T
                        marker_rpy = rotation_matrix_to_euler_angles(R_tc)
                        if self.debug:
                            logging.info(f'x:{x}, y:{y}, z:{z}')
                            logging.info(
                                f'roll:{marker_rpy[0]}, pitch:{marker_rpy[1]}, yaw:{marker_rpy[2]}'
                            )
                        valid = True
                    elif self.debug:
                        logging.info('Board not detected')
            # Display the results
            if display:
                cv2.imshow('Detected Markers', image_markers)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                if loop:
                    logging.info('Detection stopped')
                break
        if display:
            self._capture.release()
            cv2.destroyAllWindows()


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    detector = BoardDetector(debug=True)
    detector.run(loop=True, display=True)
    # main()