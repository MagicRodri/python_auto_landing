import logging
import math
from typing import Dict, Tuple, Union

from pymavlink import mavutil


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


def marker_position_to_angle(x, y, z):

    angle_x = math.atan2(x, z)
    angle_y = math.atan2(y, z)

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


def change_mode(master, mode: str = None):
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
                logging.info("Mode changed to %s"%(mode))
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
    if inputs is None:
        inputs = {}
        inputs[id] = pwm
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

def stop_rc_override(master: Union[mavutil.mavfile, mavutil.mavudp,
                                  mavutil.mavtcp]) -> None:
    """Stop RC override.
    @param master: MAVLink master instance
    """
    rc_channels_override(master, 0, 0)