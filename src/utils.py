import logging
import math
import os
from typing import Dict, Tuple, Union

import numpy as np
from pymavlink import mavutil


def rotation_matrix_to_euler_angles(R: np.ndarray) -> np.ndarray:
    # Calculates rotation matrix to euler angles
    # The result is the same as MATLAB except the order
    # of the euler angles ( x and z are swapped ).

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


def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a Location object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned Location has the same `alt and `is_relative` values
    as `original_location`.
    """
    earth_radius = 6378137.0  # Radius of "spherical" earth
    if isinstance(original_location, dict):
        original_lat = original_location.get('lat') / 1e7
        original_lon = original_location.get('lon') / 1e7
    else:
        original_lat = original_location.lat
        original_lon = original_location.lon
    # Coordinate offsets in radians
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_lat / 180))

    print("dlat, dlon", dLat, dLon)

    # New position in decimal degrees
    newlat = original_lat + (dLat * 180 / math.pi)
    newlon = original_lon + (dLon * 180 / math.pi)
    return (newlat, newlon)


def marker_position_to_angle(x: float, y: float, z: float):
    """
    Converts marker position to angle
    """
    angle_x = round(math.atan2(x, z), 2)
    angle_y = round(math.atan2(y, z), 2)

    return (angle_x, angle_y)


def camera_to_uav(x_cam: float, y_cam: float) -> Tuple[float, float]:
    x_uav = -y_cam
    y_uav = x_cam
    return (x_uav, y_uav)


def uav_to_ne(x_uav: float, y_uav: float,
              yaw_rad: float) -> Tuple[float, float]:
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)

    north = x_uav * c - y_uav * s
    east = x_uav * s + y_uav * c
    return (north, east)


def check_angle_descend(angle_x, angle_y, angle_desc):
    return (math.sqrt(angle_x**2 + angle_y**2) <= angle_desc)


def request_message_interval(master: Union[mavutil.mavfile, mavutil.mavudp,
                                           mavutil.mavtcp], message_id: int,
                             frequency_hz: float) -> None:
    """
        Function to request mav message at given frequency
    """

    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                 0, message_id, 1e6 / frequency_hz, 0, 0, 0, 0,
                                 0)


def change_mode(master: Union[mavutil.mavfile, mavutil.mavudp, mavutil.mavtcp],
                mode: str = None):
    if mode is not None:
        mode_id = master.mode_mapping().get(mode)
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
        ack_msg = master.recv_match(type="COMMAND_ACK", blocking=True)
        if ack_msg is not None:
            ack_msg = ack_msg.to_dict()
            message = mavutil.mavlink.enums["MAV_RESULT"][
                ack_msg['result']].description
            if message == "Command is valid (is supported and has valid parameters), and was executed.":
                logging.info("Mode changed to %s" % (mode))
            else:
                logging.info(message)


def rc_channels_override(master: Union[mavutil.mavfile, mavutil.mavudp,
                                       mavutil.mavtcp],
                         id: int = None,
                         pwm: int = None,
                         inputs: Dict[int, int] = None,
                         mavlink2: bool = False) -> None:
    """Override RC channels with PWM values.
    @param master: MAVLink master instance
    @param id: Channel ID
    @param pwm: PWM value
    @param inputs: Dictionary of channel IDs and PWM values
    @param mavlink2: Use MAVLink 2.0 protocol
        -By default, MAVLink 1.0 protocol is used
    """
    if id is not None and pwm is not None:
        inputs = {}
        inputs[id] = pwm
    if inputs is None:
        logging.warning('No inputs provided')
        return
    if os.environ.get('MAVLINK20') is not None:
        mavlink2 = True
    max_id = 9
    if mavlink2:
        max_id = 18
    rc_channel_values = [65535 for _ in range(max_id)]
    for channel_id, value in inputs.items():
        if channel_id not in range(1, max_id + 1):
            logging.warning('Invalid channel id: %s', channel_id)
            return
        if value not in range(1000, 1901):
            logging.warning('Invalid value: %s', value)
            return
        rc_channel_values[channel_id - 1] = value
    master.mav.rc_channels_override_send(master.target_system,
                                         master.target_component,
                                         *rc_channel_values)


def play_buzzer_tune(master: Union[mavutil.mavfile, mavutil.mavudp,
                                   mavutil.mavtcp], tune: str):
    """Function to play a tune on the buzzer.
    Parameters
    ----------
    master : mavutil.mavlink.MAVLink_connection
        MAVLink connection to the FCU.
    tune : str
        Tune to play.
    """
    # Encode the tune
    # Note: The MAVLink play_tune_encode() function expects a string
    # with UTF-8 encoding.
    # https://mavlink.io/en/messages/common.html#PLAY_TUNE
    msg = master.mav.play_tune_encode(
        master.target_system,  # Target system
        master.target_component,  # Target component
        tune.encode('utf-8'),  # Tune string
    )
    # Send the message
    master.mav.send(msg)


def send_altitude(master: Union[mavutil.mavfile, mavutil.mavudp,
                                mavutil.mavtcp], distance: int):
    """
    Send altitude to FCU to enable altitude hold
    """
    sensor_id = 1
    orientation = 25
    covariance = 70
    master.mav.distance_sensor_send(
        0, 10, 1000, distance, mavutil.mavlink.MAV_DISTANCE_SENSOR_ULTRASOUND,
        sensor_id, orientation, covariance)
    # logging.info(f"Sending rangefinder altitude: {distance}")


def send_landing_target(master: Union[mavutil.mavfile, mavutil.mavudp,
                                      mavutil.mavtcp], distance: float,
                        x_angle: float, y_angle: float, time_usec: int):
    """Function  to send landing target to FCU
    Parameters
    ----------
    master : mavutil.mavlink.MAVLink_connection
        MAVLink connection to the FCU.
    distance : float
        Distance to the target in meters.
    x_angle : float
        X angle of the target in radians.
    y_angle : float
        Y angle of the target in radians.
    time_usec : int
        Timestamp in microseconds.
    """
    # logging.info(f"Sending landing target: {distance}, {x_angle}, {y_angle}, {time_usec}")
   
    msg = master.mav.landing_target_encode(
        time_usec,  # Timestamp, not used
        0,  # Target num
        mavutil.mavlink.MAV_FRAME_BODY_NED,  # Frame,not used
        x_angle,
        y_angle,
        distance,  # Distance
        0.0,  # size_x
        0.0,  # size_y
    )
    master.mav.send(msg)
    # logging.info("sent landing target")