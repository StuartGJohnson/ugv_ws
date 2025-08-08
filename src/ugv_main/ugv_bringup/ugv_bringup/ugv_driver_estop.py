#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import json  
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import Joy
import subprocess
import time
import os
from ugv_bringup.robot_tools import setup_robot, find_robot, get_robot_serial, reset_robot
import logging

wheel_separation = 0.174
understeer_factor = 2.0

class UgvDriverEstop(Node):
    def __init__(self, name):
        super().__init__(name)

        self.logger = logging.getLogger('UgvDriverEstop')

        # subscribe to joystick input
        self.joy_sub_ = self.create_subscription(Joy, "joy", self.joy_callback, 10)

        # Subscribe to velocity commands (cmd_vel topic)
        self.cmd_vel_sub_ = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)

        # Subscribe to joint states (ugv/joint_states topic)
        self.joint_states_sub = self.create_subscription(JointState, 'ugv/joint_states', self.joint_states_callback, 10)

        # Subscribe to LED control data (ugv/led_ctrl topic)
        self.led_ctrl_sub = self.create_subscription(Float32MultiArray, 'ugv/led_ctrl', self.led_ctrl_callback, 10)

        # Subscribe to voltage data (voltage topic)
        self.voltage_sub = self.create_subscription(Float32, 'voltage', self.voltage_callback, 10)

        serial_port = find_robot()
        self.ser = get_robot_serial(serial_port)
        setup_robot(self.ser)
        self.estop = False

    # Callback for processing velocity commands
    def cmd_vel_callback(self, msg: Twist):
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # Apply minimum threshold to angular velocity if linear velocity is zero
        if linear_velocity == 0:
            if 0 < angular_velocity < 0.2:
                angular_velocity = 0.2
            elif -0.2 < angular_velocity < 0:
                angular_velocity = -0.2

        if not self.estop:
            # Send the velocity data to the UGV as a JSON string
            left_vel = linear_velocity - angular_velocity * wheel_separation * understeer_factor / 2.0
            right_vel = linear_velocity + angular_velocity * wheel_separation * understeer_factor / 2.0
            data = json.dumps({'T': '1', 'L': left_vel, 'R': right_vel}) + "\n"
            #data = json.dumps({'T': '13', 'X': linear_velocity, 'Z': angular_velocity}) + "\n"
            self.ser.write(data.encode())

    def joy_callback(self, msg: Joy):
        # update state machine state based on check of X and Y buttons
        x_button = msg.buttons[3]
        y_button = msg.buttons[4]
        if x_button > 0:
            self.logger.warning('Estop!')
            self.estop = True
            # stop!
            reset_robot(self.ser)
            return
        if y_button > 0:
            self.logger.warning('Un-Estop!')
            self.estop = False

    # Callback for processing joint state updates
    def joint_states_callback(self, msg: JointState):
        header = {
            'stamp': {
                'sec': msg.header.stamp.sec,
                'nanosec': msg.header.stamp.nanosec,
            },
            'frame_id': msg.header.frame_id,
        }

        # Extract joint positions and convert to degrees
        name = msg.name
        position = msg.position

        x_rad = position[name.index('pt_base_link_to_pt_link1')]
        y_rad = position[name.index('pt_link1_to_pt_link2')]

        x_degree = (180 * x_rad) / 3.1415926
        y_degree = (180 * y_rad) / 3.1415926

        # Send the joint data as a JSON string to the UGV
        joint_data = json.dumps({
            'T': 134, 
            'X': x_degree, 
            'Y': y_degree, 
            "SX": 600,
            "SY": 600,
        }) + "\n"

        if not self.estop:
            self.ser.write(joint_data.encode())

    # Callback for processing LED control commands
    def led_ctrl_callback(self, msg):
        IO4 = msg.data[0]
        IO5 = msg.data[1]
        
        # Send LED control data as a JSON string to the UGV
        led_ctrl_data = json.dumps({
            'T': 132, 
            "IO4": IO4,
            "IO5": IO5,
        }) + "\n"
                
        self.ser.write(led_ctrl_data.encode())

    # Callback for processing voltage data
    def voltage_callback(self, msg):
        voltage_value = msg.data

        # If voltage drops below a threshold, play a low battery warning sound
        if 0.1 < voltage_value < 9: 
            subprocess.run(['aplay', '-D', 'plughw:2,0', '/home/ws/ugv_ws/src/ugv_main/ugv_bringup/ugv_bringup/low_battery.wav'])
            time.sleep(5)

    def serial_shutdown(self):
        self.ser.close()

def main(args=None):
    rclpy.init(args=args)
    node = UgvDriverEstop("ugv_driver_estop")
    
    try:
        rclpy.spin(node)  # Keep the node running and handling callbacks
    except KeyboardInterrupt:
        pass  # Graceful shutdown on user interrupt
    finally:
        node.serial_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
