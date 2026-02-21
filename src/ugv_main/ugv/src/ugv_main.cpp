#include "ugv/ugv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // Create the node
    auto node = std::make_shared<Ugv>();

    // Use a MultiThreadedExecutor to allow callbacks in different groups to run concurrently
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin(); // Spin the executor to process callbacks
    
    rclcpp::shutdown();
    return 0;
}