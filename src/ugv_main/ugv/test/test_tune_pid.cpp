// a functional test (connected robot) which serves as a tool to collect and plot
// motor performance data. This is quite similar to ugv, but ros messages are not involvec
// and data is collected during robot operation and then plotted. I assume I have a tether
// to the robot (usb) from the computer running this test so I can move back and
// forth over a meter or so.

#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "ugv/robot_tools.hpp"
#include "ugv/base_controller.hpp"
#include <iostream>
#include <unistd.h> // For sleep
#include <nlohmann/json.hpp>
#include <atomic> // For std::atomic<bool>
#include <cstdlib> // For std::system
#include "ugv/ugv_collector.hpp"


TEST(TunePID, TestUgvCollector) {

    // set up command sequence
    std::map<int, Command> commands;
    commands[0] = CmdVel{0.0, 0.0};
    commands[1] = CmdPID{40.0, 250.0, 0.0, 0.0};
    commands[2] = CmdVel{0.4, -0.4};
    commands[3] = CmdVel{0.0, 0.0};
    commands[4] = CmdVel{-0.4, 0.4};
    commands[5] = CmdVel{0.0, 0.0};
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UgvCollector>(commands);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    std::thread spin_thread([&executor]() {
        executor.spin();
    });

    while (rclcpp::ok() && !node->is_complete()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    node->plot();

    executor.cancel();
    if (spin_thread.joinable()) {
        spin_thread.join();
    }

    executor.remove_node(node);
    rclcpp::shutdown();
}

TEST(TunePID, TestUgvCollectorMotorSpeed) {

    // run a sequence of motor speeds to calibrate
    // open loop motor control. I do this with the
    // wheels off the ground - but a brief follow-up
    // at a lower velocity under load might be handy.
    // set up command sequence
    // these are time sampling targets in seconds
    std::vector<double> targets = {3.75, 4.75, 5.75, 6.75, 7.75, 8.75, 9.75};
    std::map<int, Command> commands;
    //commands[0] = CmdVel{0.0, 0.0};
    commands[0] = CmdPID{40.0, 250.0, 0.0, 0.0};
    commands[1] = CmdFF{175.0, 10.0};
    commands[2] = CmdVel{0.1, 0.1};
    commands[3] = CmdVel{0.2, 0.2};
    commands[4] = CmdVel{0.3, 0.3};
    commands[5] = CmdVel{0.4, 0.4};
    commands[6] = CmdVel{0.5, 0.5};
    commands[7] = CmdVel{0.6, 0.6};
    commands[8] = CmdVel{0.7, 0.7};
    commands[9] = CmdVel{0.0, 0.0};
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UgvCollector>(commands);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    std::thread spin_thread([&executor]() {
        executor.spin();
    });

    while (rclcpp::ok() && !node->is_complete()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    node->plot_calibrate(targets);

    executor.cancel();
    if (spin_thread.joinable()) {
        spin_thread.join();
    }

    executor.remove_node(node);
    rclcpp::shutdown();
}

TEST(TunePID, TestUgvCollectorMotorSpeedFloor) {
    // same as above, but for the robot on the floor
    // run a sequence of motor speeds to calibrate
    // open loop motor control. I do this with the
    // wheels off the ground - but a brief follow-up
    // at a lower velocity under load might be handy.
    // set up command sequence
    // these are time sampling targets in seconds
    std::vector<double> targets = {5.75, 8.75};
    std::map<int, Command> commands;
    //commands[0] = CmdVel{0.0, 0.0};
    commands[0] = CmdPID{40.0, 250.0, 0.0, 0.0};
    commands[1] = CmdFF{175.0, 10.0};
    commands[3] = CmdVel{0.35, 0.2};
    commands[4] = CmdVel{0.35, 0.2};
    commands[5] = CmdVel{0.0, 0.0};
    commands[6] = CmdVel{-0.35, -0.2};
    commands[7] = CmdVel{-0.35, -0.2};
    commands[8] = CmdVel{0.0, 0.0};
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UgvCollector>(commands);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    std::thread spin_thread([&executor]() {
        executor.spin();
    });

    while (rclcpp::ok() && !node->is_complete()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    node->plot_calibrate(targets);

    executor.cancel();
    if (spin_thread.joinable()) {
        spin_thread.join();
    }

    executor.remove_node(node);
    rclcpp::shutdown();
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
