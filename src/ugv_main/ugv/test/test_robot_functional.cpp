#include <gtest/gtest.h>
#include "ugv/robot_tools.hpp"
#include <iostream>
#include <unistd.h> // For sleep
#include "rclcpp/rclcpp.hpp"
#include "ugv/ugv.hpp"

// The vendor and product IDs should ideally be configurable via ROS parameters.
const std::string DRIVER_VENDOR_ID = "1a86"; // Example Vendor ID
const std::string DRIVER_PRODUCT_ID = "55d3"; // Example Product ID

// This test requires a physical robot connected via serial to /dev/ttyUSB0 (or configured port)
// It is intended for functional testing, not typical unit testing automation.

const int FUNCTIONAL_BAUD_RATE = 115200;

TEST(RobotToolsFunctionalTest, SetupAndZeroVelocity) {
    std::cout << "Please ensure the robot is connected and powered on." << std::endl;

    RobotTools robot_tools;

    // Find and open serial port and setup robot
    std::string serial_port_path = RobotTools::find_robot_tty(DRIVER_VENDOR_ID, DRIVER_PRODUCT_ID);
    if (serial_port_path.empty())
    {
        printf("Robot not found with Vendor ID: %s, Product ID: %s", DRIVER_VENDOR_ID.c_str(), DRIVER_PRODUCT_ID.c_str());
        return;
    }

    if (!robot_tools.open_serial(serial_port_path, 115200))
    {
        printf("Failed to open serial port %s", serial_port_path.c_str());
        return;
    }

    std::cout << "Serial port opened successfully." << std::endl;

    // 2. Setup the robot (send initial commands)
    robot_tools.setup_robot();
    std::cout << "Robot setup commands sent." << std::endl;
    usleep(100000); // Give the robot some time to process commands and start sending feedback
    
    // send initial clock sync so we get some data back
    nlohmann::json clock_sync_cmd;
    clock_sync_cmd["T"] = "42";
    robot_tools.send_command(clock_sync_cmd);

    // 3. Send a non-zero velocity command
    nlohmann::json zero_vel_cmd;
    zero_vel_cmd["T"] = "13";
    zero_vel_cmd["X"] = 0.3;
    zero_vel_cmd["Z"] = 0.0;
    robot_tools.send_command(zero_vel_cmd);
    std::cout << "Zero velocity command sent." << std::endl;
    usleep(1000000); // Give the robot some time to respond
    zero_vel_cmd["T"] = "13";
    zero_vel_cmd["X"] = 0.0;
    zero_vel_cmd["Z"] = 0.0;
    robot_tools.send_command(zero_vel_cmd);
    std::cout << "Non-Zero velocity command sent." << std::endl;
    usleep(2000000); // Give the robot some time to respond

    // 4. Attempt to read feedback to confirm communication
    std::string response_line = robot_tools.read_line();
    std::cout << "Read from serial: " << (response_line.empty() ? "[EMPTY]" : response_line) << std::endl;
    ASSERT_FALSE(response_line.empty()) << "Did not receive any feedback from the robot.";

    // Optionally, try to parse the JSON feedback
    try {
        nlohmann::json feedback = nlohmann::json::parse(response_line);
        std::cout << "Parsed feedback: " << feedback.dump(4) << std::endl;
        // Further assertions can be added here based on expected feedback structure
        // For example:
        // ASSERT_TRUE(feedback.contains("T"));
        // ASSERT_EQ(feedback["T"], 1001); // Assuming 1001 is the feedback type
    } catch (const nlohmann::json::parse_error& e) {
        std::cerr << "Failed to parse JSON feedback: " << e.what() << std::endl;
        // Depending on strictness, this might be an ASSERT_TRUE(false)
    }

    // 5. Close the serial port
    robot_tools.close_serial();
    std::cout << "Serial port closed." << std::endl;
}

TEST(RobotToolsFunctionalTest, SendPID) {
    std::cout << "Please ensure the robot is connected and powered on." << std::endl;

    RobotTools robot_tools;
    BaseController controller(robot_tools);

    // Find and open serial port and setup robot
    std::string serial_port_path = RobotTools::find_robot_tty(DRIVER_VENDOR_ID, DRIVER_PRODUCT_ID);
    if (serial_port_path.empty())
    {
        printf("Robot not found with Vendor ID: %s, Product ID: %s", DRIVER_VENDOR_ID.c_str(), DRIVER_PRODUCT_ID.c_str());
        return;
    }

    if (!robot_tools.open_serial(serial_port_path, 115200))
    {
        printf("Failed to open serial port %s", serial_port_path.c_str());
        return;
    }

    std::cout << "Serial port opened successfully." << std::endl;

    // 2. Setup the robot (send initial commands)
    robot_tools.setup_robot();
    std::cout << "Robot setup commands sent." << std::endl;
    usleep(100000); // Give the robot some time to process commands and start sending feedback

    controller.start();
    
    // send initial clock sync so we get some data back
    nlohmann::json clock_sync_cmd;
    clock_sync_cmd["T"] = "42";
    robot_tools.send_command(clock_sync_cmd);
    usleep(100000); 

    // 3. Send a zero velocity command, then set PID
    nlohmann::json zero_vel_cmd;
    zero_vel_cmd["T"] = "13";
    zero_vel_cmd["X"] = 0.0;
    zero_vel_cmd["Z"] = 0.0;
    robot_tools.send_command(zero_vel_cmd);
    std::cout << "Zero velocity command sent." << std::endl;
    usleep(1000000); // Give the robot some time to respond

    nlohmann::json set_pid_cmd;
    set_pid_cmd["T"] = "2";
    set_pid_cmd["P"] = 250.0;
    set_pid_cmd["I"] = 1.0;
    set_pid_cmd["D"] = 5.0;
    set_pid_cmd["L"] = 0.0;
    robot_tools.send_command(set_pid_cmd);
    std::cout << "set_pid_cmd command sent." << std::endl;
    usleep(2000000); // Give the robot some time to respond

    zero_vel_cmd["T"] = "13";
    zero_vel_cmd["X"] = 0.5;
    zero_vel_cmd["Z"] = 0.0;
    robot_tools.send_command(zero_vel_cmd);
    std::cout << "non-zero velocity command sent." << std::endl;
    usleep(1000000); // how far did the robot move?

    zero_vel_cmd["T"] = "13";
    zero_vel_cmd["X"] = 0.0;
    zero_vel_cmd["Z"] = 0.0;
    robot_tools.send_command(zero_vel_cmd);
    std::cout << "Zero velocity command sent." << std::endl;
    usleep(1000000); // Give the robot some time to respond

    // display last feedback
    try {
        nlohmann::json feedback;
        controller.get_message_from_queue(feedback, false);
        std::cout << "Parsed feedback: " << feedback.dump(4) << std::endl;
        // Further assertions can be added here based on expected feedback structure
        // For example:
        // ASSERT_TRUE(feedback.contains("T"));
        // ASSERT_EQ(feedback["T"], 1001); // Assuming 1001 is the feedback type
    } catch (const nlohmann::json::parse_error& e) {
        std::cerr << "Failed to parse JSON feedback: " << e.what() << std::endl;
        // Depending on strictness, this might be an ASSERT_TRUE(false)
    }

    // 5. Close the serial port
    controller.stop();
    robot_tools.close_serial();
    std::cout << "Serial port closed." << std::endl;
}

TEST(RobotToolsFunctionalTest, TestUgv) {
    // this can be killed manually. There should
    // be one json decode on the first (partial) message from
    // the robot.
    // todo: see the next check-in on the python side
    // from the orin. In that code, telemetry errors are
    // tracked by the base_controller class. That should
    // be implemented in c++ (this ROS2 package) as well.
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Ugv>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
