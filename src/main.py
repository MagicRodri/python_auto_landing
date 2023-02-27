import logging
import random
import time

from pymavlink import mavutil

logging.basicConfig(level=logging.INFO)


def set_rc_channel_pwm(channel_id: int, pwm: int) -> None:
    if not 1 <= channel_id <= 18:
        logging.warning("Channel doesn't exist!!")
        return

    logging.info("Setting channel value...")
    rc_channel_values = [65535 for _ in range(9)]
    rc_channel_values[channel_id - 1] = pwm
    master.mav.rc_channels_override_send(master.target_system,
                                         master.target_component,
                                         *rc_channel_values)


def plus_minus(value: int, plus_minus: int) -> int:
    return value + random.choice([-1, 1]) * plus_minus


master = mavutil.mavlink_connection(device="127.0.0.1:14551",
                                    baud=57600,
                                    dialect="ardupilotmega")
master.wait_heartbeat()
# print(dir(master))
print(f'Hearbeat from ({master.target_system},{master.target_component})')
master.arducopter_arm()
master.motors_armed_wait()
print("Armed!")
# time.sleep(1)
# counter = 0
# while True:
#     logging.info("counter: %s" % (counter))
#     value = 1500
#     if counter == 10:
#         value = plus_minus(1500, 1)
#         set_rc_channel_pwm(3, value)
#         time.sleep(1)
#         continue
#     set_rc_channel_pwm(3, value)
#     time.sleep(1)
#     counter += 1

for channel in range(1, 5):
    master.mav.command_long_send(
        1,  # target component ID
        1,  # autopilot system ID
        mavutil.mavlink.MAV_CMD_DO_REPEAT_SERVO,  # command ID
        0,  # confirmation
        channel,  # servo output channel (1-4)
        1600,  # PWM value (1000-2000)
        5,
        1000,
        0,
        0,
        0  # unused parameters
    )
    time.sleep(0.1)