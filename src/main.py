"""
    Experimental rc override in stabilise mode to hold position
"""
import logging
from typing import Union
from utils import change_mode,rc_channels_override,request_message_interval
import time
import asyncio
import random

from pymavlink import mavutil
# 0 release
# 65535 ignore input
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
                rc_channels_override(master, inputs={1:0,2:0,3:0,4:0})
        await asyncio.sleep(0.01)


async def show_servos_output(master:Union[mavutil.mavfile, mavutil.mavudp,
                                  mavutil.mavtcp]):
    request_message_interval(master=master,message_id=mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,frequency_hz=10)
    while True:
        msg = master.recv_match(type="SERVO_OUTPUT_RAW",blocking=True)
        if msg:
            print(msg)

async def send_altitude(master:Union[mavutil.mavfile, mavutil.mavudp,
                                  mavutil.mavtcp]):

    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        b"RNGFND1_TYPE",
        10,
        mavutil.mavlink.MAV_PARAM_TYPE_INT8
        )
    min_measure = 10
    max_measure = 35
    sensor_type = 2
    sensor_id = 1
    orientation = 25
    covariance = 0

    start = time.time()
    while True:
        time.sleep(0.5)
        distance = 10
        master.mav.distance_sensor_send(
            int((time.time() - start) * 1000),
            min_measure,
            max_measure,
            distance,
            sensor_type,
            sensor_id,
            orientation,
            covariance
        )
        print("sent distance")



async def hover_by_override(master):
    overrode = False
    request_message_interval(master=master,
                             message_id=mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,frequency_hz=10)
    request_message_interval(master,
                         mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 8)
    
    descending = False
    ascending = False
    channel3_pwm = None
    altitude = None
    while True:
        if OVERRIDING:
            pos_msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if altitude is None and pos_msg:
                altitude = pos_msg.relative_alt / 10
            print(f'Latest altitude:{altitude}')
            current_altitude = pos_msg.relative_alt / 10
            print('Current alt:',current_altitude)
            error = current_altitude - altitude
            channels_msg = master.recv_match(type="RC_CHANNELS",blocking=True)
            if not channel3_pwm:
                channel3_pwm = channels_msg.chan3_raw
            print(channel3_pwm)
            if not overrode:
                print('overrode')
                rc_channels_override(master, inputs={3:channel3_pwm})
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
    logging.info(f'Hearbeat from ({master.target_system},{master.target_component})')
    # change_mode(master, "STABILIZE")
    # master.arducopter_arm()
    # master.motors_armed_wait()
    # logging.info("Armed!")
    # altitude_task = asyncio.create_task(send_altitude(master))
    listening_task = asyncio.create_task(listen_on_channel6(master))
    override_task = asyncio.create_task(hover_by_override(master))
    await asyncio.gather(listening_task,override_task)
    # await show_servos_output(master=master)

if __name__ == "__main__":
    asyncio.run(main())

