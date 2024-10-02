#include "tortoisebot_waypoints/tortoisebot_action_server.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);

  auto node = std::make_shared<TortoisebotActionServer>(
      "tortoisebot_as", "cmd_vel", "odom", "tortoisebot_action_server");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}