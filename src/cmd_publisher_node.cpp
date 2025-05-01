#include "rclcpp/rclcpp.hpp"
#include "cmd_publisher.h"

int main(int argc, const char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdPublisher>());
    rclcpp::shutdown();
    return 0;
}