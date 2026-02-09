#ifndef UGV_ROBOT_TOOLS_HPP_
#define UGV_ROBOT_TOOLS_HPP_

#include <string>
#include <vector>
#include "nlohmann/json.hpp"

// Note: The original Python code dynamically found the robot's TTY device.
// This C++ version requires the device path to be provided.
// A more robust solution would use libudev to find the device automatically.

class RobotTools {
public:
    RobotTools();
    ~RobotTools();

    bool open_serial(const std::string& port, int baud_rate);
    void close_serial();
    bool is_open() const;

    void setup_robot();
    void reset_robot();
    virtual void send_command(const nlohmann::json& command);
    std::string read_line();
    void clear_read_buffer();
    static std::string find_robot_tty(const std::string& vendor_id, const std::string& product_id);

private:
    int serial_fd_;
    std::vector<char> read_buffer_;
};

#endif  // UGV_ROBOT_TOOLS_HPP_
