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

import sys
import wiringpi
from wiringpi import GPIO

TRIG = 2
ECHO = 4
wiringpi.wiringPiSetup()
wiringpi.pinMode(TRIG, GPIO.OUTPUT)
wiringpi.pinMode(ECHO, GPIO.INPUT)
# print('waiting...')
# time.sleep(2)
def read_rangefinder_altitude() -> int:
    try:
        wiringpi.digitalWrite(TRIG, GPIO.HIGH)
        wiringpi.digitalWrite(TRIG, GPIO.LOW)
        while wiringpi.digitalRead(ECHO) == 0:
            start = time.time()
        while wiringpi.digitalRead(ECHO) == 1:
            end = time.time()
        duration = end - start
        distance = round(duration * 17150, 2)
        return int(distance) 
    except KeyboardInterrupt:
        print("\nexit")
        sys.exit(0)
    except:
        pass

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
            logging.info(msg)


async def send_altitude(master: Union[mavutil.mavfile, mavutil.mavudp,
                                      mavutil.mavtcp]):
    """
    Send altitude to FCU to enable altitude hold
    """
    sensor_id = 1
    orientation = 25
    covariance = 70
    logging.info('Sending rangefinder altitude')
    while True:
        distance = read_rangefinder_altitude()
        master.mav.distance_sensor_send(0, 10, 1000, distance,
                                        mavutil.mavlink.MAV_DISTANCE_SENSOR_ULTRASOUND, sensor_id, orientation,
                                        covariance)
        logging.info(f"sent {distance}")
        await asyncio.sleep(0.1)


async def set_home_position(master: Union[mavutil.mavfile, mavutil.mavudp,
                                          mavutil.mavtcp]):
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0, 1, 0,
                                 0, 0, 0, 0, 0)
    logging.info("sent set home position")


async def send_vision_pose_estimate(master: Union[mavutil.mavfile,
                                                  mavutil.mavudp,
                                                  mavutil.mavtcp]):
    start = time.time()
    while True:
        x = 0
        y = random.randint(0, 100)
        z = 0
        roll = 0
        pitch = random.randint(0, 100)
        yaw = 0
        master.mav.vision_position_estimate_send(
            int((time.time() - start) * 1000), x, y, z, roll, pitch, yaw)
        logging.info("sent vision pose estimate")
        await asyncio.sleep(0.1)


async def send_landing_target(master: Union[mavutil.mavfile,
                                           mavutil.mavudp,
                                           mavutil.mavtcp]):
    while True:
        x = 0.0
        y = 0.0
        z = 0.0
        x_offset_rad = 0.0
        y_offset_rad = 0.0
        distance = 1.0
        msg = master.mav.landing_target_encode(
        int(time.time()), # Timestamp
        0, # Target num
        mavutil.mavlink.MAV_FRAME_BODY_NED, # Frame
        x_offset_rad,
        y_offset_rad,
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
        logging.info("sent landing target")
        await asyncio.sleep(0.1)

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
            logging.info(f'Latest altitude:{altitude}')
            current_altitude = pos_msg.relative_alt / 10
            logging.info('Current alt:', current_altitude)
            error = current_altitude - altitude
            channels_msg = master.recv_match(type="RC_CHANNELS", blocking=True)
            if not channel3_pwm:
                channel3_pwm = channels_msg.chan3_raw
            logging.info(channel3_pwm)
            if not overrode:
                logging.info('overrode')
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
    master = mavutil.mavlink_connection(device="127.0.0.1:14551",dialect="common")
    master.wait_heartbeat()
    logging.info(
        f'Hearbeat from ({master.target_system},{master.target_component})')
    # change_mode(master, "STABILIZE")
    # master.arducopter_arm()
    # master.motors_armed_wait()
    # logging.info("Armed!")
    landing_target_task = asyncio.create_task(send_landing_target(master))
    send_altitude_task = asyncio.create_task(send_altitude(master))
    await asyncio.gather(landing_target_task,send_altitude_task)
    # await send_landing_target(master)

if __name__ == "__main__":
    asyncio.run(main())
