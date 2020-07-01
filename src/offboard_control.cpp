#include "px4_mpc/offboard_control.hpp"

using namespace std::chrono_literals;

OffboardControl::OffboardControl(rclcpp::Node::SharedPtr node) : node_(node)
{
  odometry_subs_ = node_->create_subscription<px4_msgs::msg::VehicleOdometry>("VehicleOdometry_PubSubTopic", std::bind(&OffboardControl::odometryCallback, this, std::placeholders::_1));
  ctrl_mode_subs_ = node_->create_subscription<px4_msgs::msg::VehicleControlMode>("VehicleControlMode_PubSubTopic", std::bind(&OffboardControl::vehicleCtrlModeCallback, this, std::placeholders::_1));

  offboard_mode_pub_ = node->create_publisher<px4_msgs::msg::OffboardControlMode>("OffboardControlMode_PubSubTopic");

  offboard_timer_ = node_->create_wall_timer(100ms, std::bind(&OffboardControl::pubOffboardTimerCallback, this));

  // Variable initialization
  offboard_mode_enabled_ = false;

  offboard_mode_msg_.ignore_position = false;

  // offboard_mode_msg_.ignore_velocity = false;
}

void OffboardControl::odometryCallback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg)
{
}

void OffboardControl::vehicleCtrlModeCallback(const px4_msgs::msg::VehicleControlMode::UniquePtr msg)
{
  offboard_mode_enabled_ = msg->flag_control_offboard_enabled;
  if (offboard_mode_enabled_)
    std::cout << "Offboard mode enabled" << std::endl;
  else
    std::cout << "Offboard mode NOT enabled" << std::endl;
}

void OffboardControl::pubOffboardTimerCallback()
{
  offboard_mode_msg_.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
  offboard_mode_pub_->publish(offboard_mode_msg_);
}
