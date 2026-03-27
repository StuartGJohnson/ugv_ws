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
    commands[1] = CmdPID{40.0, 1000.0, 0.0, 0.0};
    commands[2] = CmdVel{0.25, -0.25};
    commands[3] = CmdVel{0.0, 0.0};
    commands[4] = CmdVel{0.0, 0.0};
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

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
