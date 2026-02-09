#include "ugv/base_controller.hpp"
#include <iostream>

BaseController::BaseController(RobotTools& robot_tools)
    : robot_tools_(robot_tools), running_(false) {}

BaseController::~BaseController() {
    stop();
}

void BaseController::start() {
    running_ = true;
    read_thread_ = std::thread(&BaseController::read_thread_func, this);
}

void BaseController::stop() {
    running_ = false;
    if (read_thread_.joinable()) {
        read_thread_.join();
    }
}

void BaseController::read_thread_func() {
    while (running_) {
        std::string line = robot_tools_.read_line(); // This now blocks until a line or timeout
        if (!line.empty()) {
            try {
                nlohmann::json data = nlohmann::json::parse(line);
                std::lock_guard<std::mutex> lock(queue_mutex_);
                message_queue_.push(data);
                // Debug: std::cerr << "[BaseController DEBUG] Pushed message to queue. Queue size: " << message_queue_.size() << std::endl;
            } catch (const nlohmann::json::parse_error& e) {
                std::cerr << "JSON parse error in background thread: " << e.what() << " on line: '" << line << "'" << std::endl;
                robot_tools_.clear_read_buffer(); // Clear buffer on error
            }
        }
    }
}

bool BaseController::get_latest_message_from_queue(nlohmann::json& data) {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (message_queue_.empty()) {
        return false;
    }

    // Discard all but the latest message
    while (message_queue_.size() > 1) {
        message_queue_.pop();
    }

    data = message_queue_.front();
    message_queue_.pop();
    // Debug: std::cerr << "[BaseController DEBUG] Pulled latest message from queue. Queue size: " << message_queue_.size() << std::endl;
    return true;
}
