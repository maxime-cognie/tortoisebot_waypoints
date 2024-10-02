#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/impl/utils.h"
#include "tortoisebot_waypoints/action/waypoint_action.hpp"
#include "gtest/gtest.h"
#include <future>
#include <memory>
#include <ostream>
#include <stdexcept>
#include <string>
#include <thread>

using std::placeholders::_1;
using namespace std::chrono_literals;

using Waypoint = tortoisebot_waypoints::action::WaypointAction;

class RclCppFixture {
public:
  RclCppFixture() { rclcpp::init(0, nullptr); }
  ~RclCppFixture() { rclcpp::shutdown(); }
};
RclCppFixture g_rclcppfixture;

class TortoisebotWaypointsTestFixture : public ::testing::Test {
public:
  TortoisebotWaypointsTestFixture() {
    node_ = rclcpp::Node::make_shared("tortoisebot_as_test");
    RCLCPP_INFO(node_->get_logger(), "Creating Test Node");

    client_ = rclcpp_action::create_client<Waypoint>(
        node_->get_node_base_interface(), node_->get_node_graph_interface(),
        node_->get_node_logging_interface(),
        node_->get_node_waitables_interface(), "tortoisebot_as");

    sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10,
        [&](nav_msgs::msg::Odometry::SharedPtr msg) { odom_ = msg; });
  }

  ~TortoisebotWaypointsTestFixture() { node_.reset(); }

  rclcpp_action::ClientGoalHandle<Waypoint>::SharedPtr
  send_goal(Waypoint::Goal &goal);

  rclcpp_action::Client<Waypoint>::WrappedResult
  get_result(rclcpp_action::ClientGoalHandle<Waypoint>::SharedPtr goal_handle);

  geometry_msgs::msg::Point get_pos();

  float get_yaw();

protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<Waypoint>::SharedPtr client_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  nav_msgs::msg::Odometry::SharedPtr odom_;
};

rclcpp_action::ClientGoalHandle<Waypoint>::SharedPtr
TortoisebotWaypointsTestFixture::send_goal(Waypoint::Goal &goal) {
  auto goal_handle_future = client_->async_send_goal(goal);

  auto result =
      rclcpp::spin_until_future_complete(node_, goal_handle_future, 10s);
  if (result == rclcpp::FutureReturnCode::TIMEOUT) {
    RCLCPP_INFO(node_->get_logger(),
                "Timeout, unable to send the goal to the server");
    throw std::runtime_error("Operation timed out after 10 seconds");
  }
  if (result == rclcpp::FutureReturnCode::SUCCESS) {
    return goal_handle_future.get();
  }
}

rclcpp_action::Client<Waypoint>::WrappedResult
TortoisebotWaypointsTestFixture::get_result(
    rclcpp_action::ClientGoalHandle<Waypoint>::SharedPtr goal_handle) {
  auto wrapped_result_future = client_->async_get_result(goal_handle);

  auto result =
      rclcpp::spin_until_future_complete(node_, wrapped_result_future, 90s);
  if (result == rclcpp::FutureReturnCode::TIMEOUT) {
    RCLCPP_INFO(node_->get_logger(),
                "Timeout, unable to reach the goal within 1'30");
    throw std::runtime_error("Operation timed out after 1'30");
  }
  if (result == rclcpp::FutureReturnCode::SUCCESS) {
    return wrapped_result_future.get();
  }
}

geometry_msgs::msg::Point TortoisebotWaypointsTestFixture::get_pos() {
  return odom_->pose.pose.position;
}

float TortoisebotWaypointsTestFixture::get_yaw() {
  tf2::Quaternion q(
      odom_->pose.pose.orientation.x, odom_->pose.pose.orientation.y,
      odom_->pose.pose.orientation.z, odom_->pose.pose.orientation.w);
  return tf2::impl::getYaw(q);
}

TEST_F(TortoisebotWaypointsTestFixture, TestSendGoalSuccess) {
  Waypoint::Goal goal;
  goal.position.x = std::stof(std::getenv("GOAL_X"));
  goal.position.y = std::stof(std::getenv("GOAL_Y"));
  goal.position.z = std::stof(std::getenv("GOAL_YAW"));

  // Test if the connection to the server is established
  ASSERT_TRUE(client_->wait_for_action_server(std::chrono::seconds(10)));

  // Send the goal to the tortoisebot action-server
  rclcpp_action::ClientGoalHandle<Waypoint>::SharedPtr goal_handle;
  EXPECT_NO_THROW(goal_handle = send_goal(goal));
  // Test that the goal has been accepted
  ASSERT_NE(goal_handle, nullptr);

  // 
  rclcpp_action::Client<Waypoint>::WrappedResult wrapped_result;
  EXPECT_NO_THROW(wrapped_result = get_result(goal_handle));
  // Test that the robot reachs the goal
  EXPECT_TRUE(wrapped_result.result);

  // spin the node to receive odom data
  auto start = node_->now();
  while (node_->now() - start < 1.5s) {
    rclcpp::spin_some(node_);
  }

  // Test whether the final position of the robot is in the range of the goal
  auto pos = get_pos();
  RCLCPP_INFO(node_->get_logger(), "Final pos (%f, %f)", pos.x, pos.y);
  EXPECT_NEAR(goal.position.x, pos.x, 0.1);
  EXPECT_NEAR(goal.position.y, pos.y, 0.1);

  // Test whether the final oriention of the robot is in the range of the goal
  float yaw = get_yaw();
  RCLCPP_INFO(node_->get_logger(), "Final yaw: %f", yaw);
  EXPECT_NEAR(goal.position.z, yaw, 0.2);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}