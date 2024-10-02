#ifndef TORTOISEBOT_ACTION_SERVER_HPP_
#define TORTOISEBOT_ACTION_SERVER_HPP_

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tortoisebot_waypoints/action/waypoint_action.hpp"

using Waypoint = tortoisebot_waypoints::action::WaypointAction;
using GHWaypoint = rclcpp_action::ServerGoalHandle<Waypoint>;

class TortoisebotActionServer : public rclcpp::Node {
public:
  TortoisebotActionServer(const std::string &action_name,
                          const std::string &pub_topic_name,
                          const std::string &sub_topic_name,
                          const std::string &node_name);

private:
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const Waypoint::Goal> goal);
  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GHWaypoint> goal_handle);
  void handle_accepted(const std::shared_ptr<GHWaypoint> goal_handle);
  void execute(const std::shared_ptr<GHWaypoint> goal_handle);

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp_action::Server<Waypoint>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  geometry_msgs::msg::Point current_pos_, desired_pos_;
  float yaw_precision_, dist_precision_;
  std::string state_;
};
#endif // TORTOISEBOT_ACTION_SERVER_HPP_