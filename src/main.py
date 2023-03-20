"""
    Experimental rc override in stabilise mode to hold position
"""
import asyncio
import logging
import os
import random
import time
from typing import Union

from pymavlink import mavutil

from utils import change_mode, rc_channels_override, request_message_interval

os.environ["MAVLINK20"] = "1"

OVERRIDING = False
UINT16_MAX = 65535


async def listen_on_channel6(master):
    global OVERRIDING
    while True:
        msg = master.recv_match(type='RC_CHANNELS', blocking=True)
        if msg is not None:
            if msg.chan6_raw > 1500:
                print("OVERRIDING")
                OVERRIDING = True
            else:
                print("NOT OVERRIDING")
                OVERRIDING = False
                # Disable override
                # 0 release
                # 65535 ignore input
                rc_channels_override(master, inputs={1: 0, 2: 0, 3: 0, 4: 0})
        await asyncio.sleep(0.01)


async def show_servos_output(master: Union[mavutil.mavfile, mavutil.mavudp,
                                           mavutil.mavtcp]):
    request_message_interval(
        master=master,
        message_id=mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
        frequency_hz=10)
    while True:
        msg = master.recv_match(type="SERVO_OUTPUT_RAW", blocking=True)
        if msg:
            print(msg)


async def send_altitude(master: Union[mavutil.mavfile, mavutil.mavudp,
                                      mavutil.mavtcp]):
    """
    Send altitude to FCU to enable altitude hold
    """

    master.mav.param_set_send(master.target_system, master.target_component,
                              b"RNGFND1_TYPE", 10,
                              mavutil.mavlink.MAV_PARAM_TYPE_INT8)
    min_measure = 10
    max_measure = 350
    sensor_type = mavutil.mavlink.MAV_DISTANCE_SENSOR_ULTRASOUND
    sensor_id = 1
    orientation = 25
    covariance = 70

    while True:
        distance = random.randint(20, 23)
        master.mav.distance_sensor_send(0, min_measure, max_measure, distance,
                                        sensor_type, sensor_id, orientation,
                                        covariance)
        print(f"sent {distance}")
        await asyncio.sleep(0.1)


async def set_home_position(master: Union[mavutil.mavfile, mavutil.mavudp,
                                          mavutil.mavtcp]):
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0, 1, 0,
                                 0, 0, 0, 0, 0)
    print("sent set home position")


async def send_vision_pose_estimate(master: Union[mavutil.mavfile,
                                                  mavutil.mavudp,
                                                  mavutil.mavtcp]):
    x = 0
    y = random.randint(0, 100)
    z = 0
    roll = 0
    pitch = random.randint(0, 100)
    yaw = 0
    start = time.time()
    while True:
        master.mav.vision_position_estimate_send(
            int((time.time() - start) * 1000), x, y, z, roll, pitch, yaw)
        print("sent vision pose estimate")
        await asyncio.sleep(0.13)


async def hover_by_override(master):
    overrode = False
    request_message_interval(
        master=master,
        message_id=mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
        frequency_hz=10)
    request_message_interval(
        master, mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 8)
    channel3_pwm = None
    altitude = None
    while True:
        if OVERRIDING:
            pos_msg = master.recv_match(type='GLOBAL_POSITION_INT',
                                        blocking=True)
            if altitude is None and pos_msg:
                altitude = pos_msg.relative_alt / 10
            print(f'Latest altitude:{altitude}')
            current_altitude = pos_msg.relative_alt / 10
            print('Current alt:', current_altitude)
            error = current_altitude - altitude
            channels_msg = master.recv_match(type="RC_CHANNELS", blocking=True)
            if not channel3_pwm:
                channel3_pwm = channels_msg.chan3_raw
            print(channel3_pwm)
            if not overrode:
                print('overrode')
                rc_channels_override(master, inputs={3: channel3_pwm})
                overrode = True
            if abs(error) != 0:
                overrode = False
                channel3_pwm += int(error * 10)
        else:
            overrode = False
            channel3_pwm = None
            altitude = None
        await asyncio.sleep(0.01)


async def main():
    logging.basicConfig(level=logging.INFO)
    master = mavutil.mavlink_connection(device="127.0.0.1:14551")
    master.wait_heartbeat()
    logging.info(
        f'Hearbeat from ({master.target_system},{master.target_component})')
    # change_mode(master, "STABILIZE")
    # master.arducopter_arm()
    # master.motors_armed_wait()
    # logging.info("Armed!")
    # listening_task = asyncio.create_task(listen_on_channel6(master))
    # override_task = asyncio.create_task(hover_by_override(master))
    # await asyncio.gather(listening_task,override_task)
    await send_altitude(master)


if __name__ == "__main__":
    asyncio.run(main())
