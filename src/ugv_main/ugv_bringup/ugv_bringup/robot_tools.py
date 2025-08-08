
#!/usr/bin/env python3
# 2025 Stuart Johnson
#
# find and reset the robot
# send zero velocity cmds for some secs
# reset robot to non-arm mode.

import os
import subprocess
import serial
import json

def find_tty_by_usb_id(vendor_id, product_id):
    for dev in os.listdir("/dev"):
        if not dev.startswith("ttyUSB") and not dev.startswith("ttyACM"):
            continue
        path = os.path.join("/dev", dev)
        try:
            output = subprocess.check_output(["udevadm", "info", "-q", "property", "-n", path]).decode()
            if f"ID_VENDOR_ID={vendor_id}" in output and f"ID_MODEL_ID={product_id}" in output:
                return path
        except subprocess.CalledProcessError:
            pass
    return None

def find_robot() -> str:
    # lsusb shows my robot to be vendor 1a86, device 55d3
    # todo - put this in a config file
    return find_tty_by_usb_id("1a86", "55d3")


def get_robot_serial(robot_tty: str):
    # Initialize serial communication with the UGV
    return serial.Serial(robot_tty, 115200, timeout=1)

def setup_robot(ser: serial.Serial):
    # Set no arm; Start continuous feedback
    data = json.dumps({'T': '4', 'cmd': 0}) + "\n"
    ser.write(data.encode())
    data = json.dumps({'T': '131', 'cmd': 1}) + "\n"
    ser.write(data.encode())

def reset_robot(ser: serial.Serial):
    setup_robot(ser)
    # send zero commands

    linear_velocity = 0.0
    angular_velocity = 0.0

    # repeat a command to zero velocity to defeat the
    # robot pid loop
    for i in range(20):
        # Send the velocity data to the UGV as a JSON string
        data = json.dumps({'T': '13', 'X': linear_velocity, 'Z': angular_velocity}) + "\n"
        ser.write(data.encode())
