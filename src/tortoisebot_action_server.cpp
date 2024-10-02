#include "tortoisebot_waypoints/tortoisebot_action_server.h"

#include <cmath>
#include <memory>
#include <utility>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/impl/utils.h"

TortoisebotActionServer::TortoisebotActionServer(
    const std::string &action_name, 
    const std::string &pub_topic_name,
    const std::string &sub_topic_name, 
    const std::string &node_name)
    : Node(node_name), yaw_precision_(M_PI / 90), dist_precision_(0.02),
      state_("init") {

  using namespace std::placeholders;

  this->action_server_ = rclcpp_action::create_server<Waypoint>(
      this, action_name,
      std::bind(&TortoisebotActionServer::handle_goal, this, _1, _2),
      std::bind(&TortoisebotActionServer::handle_cancel, this, _1),
      std::bind(&TortoisebotActionServer::handle_accepted, this, _1));

  this->cmd_vel_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>(pub_topic_name, 10);

  this->odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      sub_topic_name, 10,
      std::bind(&TortoisebotActionServer::odom_callback, this, _1));
}

rclcpp_action::GoalResponse TortoisebotActionServer::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const Waypoint::Goal> goal) {
  RCLCPP_INFO(this->get_logger(),
              "Received goal request with pos:\nX = %f, Y = %f and yaw = %f",
              goal->position.x, goal->position.y, goal->position.z);
  this->desired_pos_ = goal->position;
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TortoisebotActionServer::handle_cancel(
    const std::shared_ptr<GHWaypoint> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TortoisebotActionServer::handle_accepted(
    const std::shared_ptr<GHWaypoint> goal_handle) {
  using namespace std::placeholders;
  std::thread{std::bind(&TortoisebotActionServer::execute, this, _1),
              goal_handle}
      .detach();
}

void TortoisebotActionServer::execute(
    const std::shared_ptr<GHWaypoint> goal_handle) {
  rclcpp::Rate loop_rate(30);

  auto feedback = std::make_shared<Waypoint::Feedback>();
  auto result = std::make_shared<Waypoint::Result>();
  result->success = true;
  auto cmd_vel = geometry_msgs::msg::Twist();

  float desired_yaw = std::atan2(this->desired_pos_.y - this->current_pos_.y,
                                 this->desired_pos_.x - this->current_pos_.x);
  float error_yaw = desired_yaw - this->current_pos_.z;
  float error_pos =
      std::sqrt((this->desired_pos_.x - this->current_pos_.x) *
                    (this->desired_pos_.x - this->current_pos_.x) +
                (this->desired_pos_.y - this->current_pos_.y) *
                    (this->desired_pos_.y - this->current_pos_.y));

  while (error_pos > dist_precision_) {
    loop_rate.sleep();
    desired_yaw = std::atan2(this->desired_pos_.y - this->current_pos_.y,
                             this->desired_pos_.x - this->current_pos_.x);
    error_yaw = desired_yaw - this->current_pos_.z;
    error_yaw = std::abs(error_yaw)>M_PI?
                (desired_yaw<0)?
                -((2*M_PI + desired_yaw) - this->current_pos_.z):
                desired_yaw - (2*M_PI + this->current_pos_.z):
                error_yaw;
    error_pos = std::sqrt((this->desired_pos_.x - this->current_pos_.x) *
                              (this->desired_pos_.x - this->current_pos_.x) +
                          (this->desired_pos_.y - this->current_pos_.y) *
                              (this->desired_pos_.y - this->current_pos_.y));

    RCLCPP_INFO(this->get_logger(), "Current Yaw: %f", this->current_pos_.z);
    RCLCPP_INFO(this->get_logger(), "Desired Yaw: %f", desired_yaw);
    RCLCPP_INFO(this->get_logger(), "Error Yaw: %f", error_yaw);

    if (goal_handle->is_canceling()) {
      result->success = false;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }
    if (std::abs(error_yaw) > 8 * M_PI / 180) {
      RCLCPP_INFO(this->get_logger(), "Fix yaw");
      this->state_ = "fix yaw";
      cmd_vel.linear.x = 0;
      cmd_vel.angular.z = error_yaw > 0 ? 1.0 : -1.0;
    } else {
      RCLCPP_INFO(this->get_logger(), "Go to point");
      this->state_ = "go to point";
      cmd_vel.linear.x = 0.2;
      cmd_vel.angular.z = 0;
    }
    this->cmd_vel_pub_->publish(cmd_vel);
    feedback->position = this->current_pos_;
    feedback->state = this->state_;
    goal_handle->publish_feedback(feedback);
  }

  cmd_vel.linear.x = 0;
  cmd_vel.angular.z = 0;
  this->cmd_vel_pub_->publish(cmd_vel);

  error_yaw = this->desired_pos_.z - this->current_pos_.z;
  while (std::abs(error_yaw) > this->yaw_precision_) {
    if (goal_handle->is_canceling()) {
      result->success = false;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }
    error_yaw = this->desired_pos_.z - this->current_pos_.z;
    error_yaw = std::abs(error_yaw)>M_PI?
                (this->desired_pos_.z<0)?
                -((2*M_PI + this->desired_pos_.z) - this->current_pos_.z):
                this->desired_pos_.z - (2*M_PI + this->current_pos_.z):
                error_yaw;

    RCLCPP_INFO(this->get_logger(), "Current Yaw: %f", this->current_pos_.z);
    RCLCPP_INFO(this->get_logger(), "Desired Yaw: %f", desired_pos_.z);
    RCLCPP_INFO(this->get_logger(), "Error Yaw: %f", error_yaw);
    RCLCPP_INFO(this->get_logger(), "Fix yaw");
    this->state_ = "fix yaw";
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = error_yaw > 0 ? 1.0 : -1.0;
    this->cmd_vel_pub_->publish(cmd_vel);
    feedback->position = this->current_pos_;
    feedback->state = this->state_;
    goal_handle->publish_feedback(feedback);
    loop_rate.sleep();
  }

  cmd_vel.linear.x = 0;
  cmd_vel.angular.z = 0;
  this->cmd_vel_pub_->publish(cmd_vel);

  if (rclcpp::ok()) {
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
}

void TortoisebotActionServer::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  this->current_pos_.x = msg->pose.pose.position.x;
  this->current_pos_.y = msg->pose.pose.position.y;
  tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  this->current_pos_.z = tf2::impl::getYaw(q);
}