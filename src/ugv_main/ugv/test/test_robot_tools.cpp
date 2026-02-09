#include <gtest/gtest.h>
#include "ugv/robot_tools.hpp"
#include <nlohmann/json.hpp>

// A mock class for RobotTools to avoid actual serial communication in tests.
class MockRobotTools : public RobotTools {
public:
    std::vector<nlohmann::json> sent_commands;

    // Override send_command to store commands instead of sending them.
    void send_command(const nlohmann::json& command) override {
        sent_commands.push_back(command);
    }
};

TEST(RobotToolsTest, SetupRobot) {
    MockRobotTools robot_tools;
    robot_tools.setup_robot();

    ASSERT_EQ(robot_tools.sent_commands.size(), 2);

    EXPECT_EQ(robot_tools.sent_commands[0]["T"], "4");
    EXPECT_EQ(robot_tools.sent_commands[0]["cmd"], 0);

    EXPECT_EQ(robot_tools.sent_commands[1]["T"], "131");
    EXPECT_EQ(robot_tools.sent_commands[1]["cmd"], 1);
}

TEST(RobotToolsTest, ResetRobot) {
    MockRobotTools robot_tools;
    robot_tools.reset_robot();

    // 2 for setup + 20 for zero velocity
    ASSERT_EQ(robot_tools.sent_commands.size(), 22);

    // Check the first two setup commands
    EXPECT_EQ(robot_tools.sent_commands[0]["T"], "4");
    EXPECT_EQ(robot_tools.sent_commands[0]["cmd"], 0);
    EXPECT_EQ(robot_tools.sent_commands[1]["T"], "131");
    EXPECT_EQ(robot_tools.sent_commands[1]["cmd"], 1);

    // Check one of the zero velocity commands
    EXPECT_EQ(robot_tools.sent_commands[2]["T"], "13");
    EXPECT_EQ(robot_tools.sent_commands[2]["X"], 0.0);
    EXPECT_EQ(robot_tools.sent_commands[2]["Z"], 0.0);
}

// TODO: The user can add more tests here to send a predetermined set of commands.
// Example:
// TEST(RobotToolsTest, CustomCommandSequence) {
//     MockRobotTools robot_tools;
//
//     nlohmann::json custom_cmd;
//     custom_cmd["T"] = "my_command";
//     custom_cmd["value"] = 42;
//     robot_tools.send_command(custom_cmd);
//
//     ASSERT_EQ(robot_tools.sent_commands.size(), 1);
//     EXPECT_EQ(robot_tools.sent_commands[0], custom_cmd);
// }

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
