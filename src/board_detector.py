import logging
import math
import sys
from pathlib import Path
from typing import Tuple, Union

import cv2
import numpy as np
import yaml


class BoardDetector:
    """Board detector class"""

    def __init__(self,
                 board_layout_path: Union[str,Path],
                 calibration_path: Union[str,Path],
                 *,
                 detector_parameters_path: Union[str,Path] = None,
                 video_device: int = 0,
                 resolution: Tuple[int] = (640, 480),
                 debug: bool = False) -> None:
        self.video_device = video_device
        self.resolution = resolution
        self.debug = debug

        self._capture = cv2.VideoCapture(self.video_device)
        self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
        self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])

        # Load board data
        self.board_corners, self.board_ids = self._load_board_data(
            board_layout_path)

        # Load camera parameters
        self.camera_matrix, self.distortion_coeffs = self._load_camera_parameters(
            calibration_path)

        # Define Aruco dictionary and board parameters
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)

        self.board = cv2.aruco.Board_create(self.board_corners,
                                            self.aruco_dict, self.board_ids)

        # Load detector parameters
        self.detector_params = self._load_detector_parameters(
            detector_parameters_path)
        self.detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

    def _is_rotation_matrix(self, R):
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype=R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6

    def _rotation_matrix_to_euler_angles(self, R: np.array) -> np.array:
        """
        """
        assert (self._is_rotation_matrix(R))

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

    def _load_board_data(self, file_path: Union[str,Path]) -> Tuple[np.array]:
        """ Load board's data """
        with open(file_path, 'r') as file:
            data = yaml.load(file, Loader=yaml.SafeLoader)
            mm_px = data['mm_per_unit'] * 0.001
            corners = np.array(data['corners']).astype(np.float32) * mm_px
            ids = np.array(data['ids'])
            return corners, ids

    def _load_camera_parameters(self,
                                file_path: Union[str,Path]) -> Tuple[np.array]:
        """ Load camera's calibration data """
        with open(file_path, 'r') as file:
            data = yaml.load(file, Loader=yaml.SafeLoader)
            camera_matrix = np.array(data['camera_matrix'])
            distortion_coeffs = np.array(data['distortion_coeffs'])
            return camera_matrix, distortion_coeffs

    def _load_detector_parameters(
            self, file_path: Union[str,Path]) -> cv2.aruco_DetectorParameters:
        """ Load detector parameters from yaml file """
        params = cv2.aruco.DetectorParameters_create()
        if file_path is not None:
            with open(file_path, 'r') as file:
                data = yaml.load(file, Loader=yaml.SafeLoader)
                for key, value in data.items():
                    setattr(params, key, value)
        return params

    def _log_info(self, text: str):
        if self.debug:
            logging.info(text)

    def run(self, loop: bool = True, display: bool = False) -> Union[None,Tuple]:
        """Main method responsible of running detection
           Returns None if loop = True
                   tuple(bool,np.array,np.array) if loop = False
        """
        camera_coordinates = np.array([0, 0, 0])
        camera_rpy = np.array([0, 0, 0])
        if loop:
            logging.info('Starting detection...')

        while True:
            ret, frame = self._capture.read()
            if not ret:
                break

            # Convert the frame to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Detect the markers in the frame
            # returns corners, ids and rejected
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.board.dictionary, parameters=self.detector_params)

            # Draw the detected markers candidates
            image_markers = cv2.aruco.drawDetectedMarkers(
                frame.copy(), corners, ids)

            # Check if the detected markers correspond to the board's corners and ids
            found = False
            if ids is not None and len(ids) > 0:
                self._log_info(f'Detected {ids.size} markers')
                valid_ids = []
                valid_corners = []

                for i in range(len(ids)):
                    if ids[i][0] in self.board_ids:
                        valid_ids.append(ids[i][0])
                        valid_corners.append(corners[i])

                if len(valid_ids) > 0:
                    _, rvec, tvec = cv2.aruco.estimatePoseBoard(
                        np.array(valid_corners), np.array(valid_ids),
                        self.board, self.camera_matrix, self.distortion_coeffs,
                        None, None)

                    if cv2.norm(tvec) > 0.00001:
                        if display:
                            cv2.aruco.drawAxis(image_markers,
                                               self.camera_matrix,
                                               self.distortion_coeffs, rvec,
                                               tvec, 0.4)
                        # Camera's coordinates in board's coordinate system (millimeters)
                        camera_coordinates = tvec.flatten()
                        # the rotation matrix tag->camera
                        R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
                        R_tc = R_ct.T
                        camera_rpy = self._rotation_matrix_to_euler_angles(
                            R_tc)
                        self._log_info(
                            f'x:{camera_coordinates[0]}, y:{camera_coordinates[1]}, z:{camera_coordinates[2]}'
                        )
                        self._log_info(
                            f'roll:{camera_rpy[0]}, pitch:{camera_rpy[1]}, yaw:{camera_rpy[2]}'
                        )
                        found = True
                    else:
                        self._log_info('Board not detected')
            # Return result if not detecting in a loop
            if not loop:
                return found, camera_coordinates, camera_rpy
            # Display the results
            if display:
                cv2.imshow('Detected Markers', image_markers)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                if loop:
                    logging.info('Aborting detection...')
                break
        if display:
            self._capture.release()
            cv2.destroyAllWindows()


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    BASE_DIR = Path(sys.argv[0]).resolve().parent
    BOARD_DATA_DIR = BASE_DIR / 'board_data'
    CALIBRATION_DIR = BASE_DIR / 'calibration_data'
    detector = BoardDetector(
        board_layout_path=BOARD_DATA_DIR / 'board_layout.yaml',
        calibration_path=CALIBRATION_DIR / 'calibration.yaml',
        detector_parameters_path=BOARD_DATA_DIR / 'detector_params.yaml',
        debug=True,
        video_device=0)

    detector.run(loop=True, display=False)