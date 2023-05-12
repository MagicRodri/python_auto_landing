from utils import rc_channels_override
from pymavlink import mavutil
import argparse


if __name__ == "__main__":
    master = mavutil.mavlink_connection(device="127.0.0.1:14551")
    master.wait_heartbeat()
    print(f'Hearbeat from ({master.target_system},{master.target_component})')
    # parser = argparse.ArgumentParser()
    # parser.add_argument("--channel", type=int, default=6)
    # parser.add_argument("--pwm", type=int, default=1500)
    # args = parser.parse_args()
    # rc_channels_override(master, args.channel, args.pwm)
