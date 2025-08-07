#include "fast_lio_localization_sc_qn.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FastLioLocalizationScQn>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    // Spin the node until shutdown
    executor.spin();

    // Shutdown ROS2
    rclcpp::shutdown();

    return 0;
}
