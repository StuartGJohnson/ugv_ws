import unittest
from ugv_bringup.ugv_bringup import BaseController
from ugv_bringup.robot_tools import setup_robot, find_robot, get_robot_serial, reset_robot

class MyTestCase(unittest.TestCase):
    def test_something(self):
        # Initialize the base controller with the UART port and baud rate
        serial_port = find_robot()
        self.ser = get_robot_serial(serial_port)
        setup_robot(self.ser)
        reset_robot(self.ser)
        self.base_controller = BaseController(self.ser)
        self.base_controller.feedback_data()
        print(self.base_controller.base_data)
        #self.assertEqual(True, False)  # add assertion here

if __name__ == '__main__':
    unittest.main()
