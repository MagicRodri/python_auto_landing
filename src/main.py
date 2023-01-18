import logging
import time

from pymavlink import mavutil

logging.basicConfig(level=logging.INFO)


def arm(master):
    # master.arducopter_arm()
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                 0, 1, 0, 0, 0, 0, 0, 0)

    logging.info("Waiting the vehicle to arm")
    master.motors_armed_wait()
    logging.info("Armed!")


def disarm(master):
    # master.arducopter_disarm()
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                 0, 0, 0, 0, 0, 0, 0, 0)

    logging.info("Waiting the vehicle to disarm")
    master.motors_disarmed_wait()
    logging.info("Disarmed!")


def change_mode(mode: str = None):
    if mode is not None:
        mode_id = master.mode_mapping().get(mode)

        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
        time.sleep(4)
        # counter = 0
        # while True:

        #     ack_msg = master.recv_match(type="COMMAND_ACK",blocking=True)
        #     ack_msg = ack_msg.to_dict()
        #     time.sleep(1)
        #     counter+=1
        #     if counter == 5:
        #         break
        #     if ack_msg['command'] != mavutil.mavlink.MAV_CMD_DO_SET_MODE:
        #         continue
        #     print(mavutil.mavlink.enums["MAV_RESULT"][ack_msg['result']].description)
        #     break


def manual_control(master):
    counter = 0
    while True:
        logging.info("Sending manual...")
        counter += 1
        master.mav.manual_control_send(master.target_system, 500, -500, 250,
                                       500, 0)
        time.sleep(1)
        if counter == 3:
            break


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


master = mavutil.mavlink_connection(device="127.0.0.1:14551",
                                    baud=57600,
                                    dialect="ardupilotmega")
master.wait_heartbeat()
# print(dir(master))
print(f'Hearbeat from ({master.target_system},{master.target_component})')
# master.arducopter_arm()
# while True:
#     msg = master.recv_match(type="COMMAND_ACK", blocking=True)
#     msg = msg.to_dict()
#     if msg['command'] != mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
#         continue
#     print(mavutil.mavlink.enums["MAV_RESULT"][msg['result']].description)
#     break
set_rc_channel_pwm(5, 1400)
