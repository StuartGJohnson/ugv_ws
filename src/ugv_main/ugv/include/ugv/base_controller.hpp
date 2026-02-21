#ifndef UGV_BASE_CONTROLLER_HPP_
#define UGV_BASE_CONTROLLER_HPP_

#include "ugv/robot_tools.hpp"
#include <thread>
#include <mutex>
#include <queue>
#include <atomic>
#include "nlohmann/json.hpp"

class BaseController {
public:
    BaseController(RobotTools& robot_tools);
    ~BaseController();

    void start();
    void stop();
    bool get_message_from_queue(nlohmann::json& data);

private:
    void read_thread_func();

    RobotTools& robot_tools_;
    std::thread read_thread_;
    std::mutex queue_mutex_;
    std::queue<nlohmann::json> message_queue_;
    std::atomic<bool> running_;
};

#endif  // UGV_BASE_CONTROLLER_HPP_
