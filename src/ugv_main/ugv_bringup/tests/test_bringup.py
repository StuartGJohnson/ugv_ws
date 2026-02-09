import unittest
import time
from ugv_bringup.ugv_bringup import BaseController, ugv_bringup
from ugv_bringup.robot_tools import setup_robot, find_robot, get_robot_serial, reset_robot
import rclpy
from rclpy.node import Node

class MyTestCase(unittest.TestCase):
    def test_something(self):
        # Initialize the base controller with the UART port and baud rate
        serial_port = find_robot()
        ser = get_robot_serial(serial_port)
        setup_robot(ser)
        reset_robot(ser)
        base_controller = BaseController(ser)
        time.sleep(1)
        base_controller.feedback_data()
        # this will typically have a json error - the robot
        # is typically running before the base_controller is created
        print("msg count: " + str(base_controller.msg_count))
        print("bad msg count: " + str(base_controller.bad_msg_count))
        print("last bad msg: " + str(base_controller.last_bad_msg))

    def test_node(self):
        rclpy.init()  # Initialize ROS
        node = ugv_bringup()  # Create the UGV bringup node
        start = time.time()
        timeout = 5.0
        while rclpy.ok() and (time.time() - start) < timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
        # one should expect to see a single bad json decode on the first
        # message (only)
        print("msg count: " + str(node.base_controller.msg_count))
        print("bad msg count: " + str(node.base_controller.bad_msg_count))
        print("last bad msg: " + str(node.base_controller.last_bad_msg))
        node.serial_shutdown()
        node.destroy_node()
        rclpy.shutdown()  # Shutdown ROS

if __name__ == '__main__':
    unittest.main()
