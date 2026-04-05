#pragma once
#include "rclcpp/rclcpp.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

// Message types
using TrajectoryMsg = autoware_auto_planning_msgs::msg::Trajectory;
using GoalMsg = geometry_msgs::msg::PoseStamped;
using TfMsg = tf2_msgs::msg::TFMessage;

namespace rsu_autoware_bridge
{
class VehicleBridgeNode : public rclcpp::Node
{
public:
  explicit VehicleBridgeNode(const rclcpp::NodeOptions & options);

private:
  // === Subscribers from Autoware ===
  rclcpp::Subscription<TrajectoryMsg>::SharedPtr sub_autoware_traj_;
  rclcpp::Subscription<TfMsg>::SharedPtr sub_autoware_tf_;
  rclcpp::Subscription<GoalMsg>::SharedPtr sub_autoware_goal_;

  // === Publishers to RSU (V2X) ===
  rclcpp::Publisher<TrajectoryMsg>::SharedPtr pub_v2x_traj_;
  rclcpp::Publisher<TfMsg>::SharedPtr pub_v2x_tf_;
  rclcpp::Publisher<GoalMsg>::SharedPtr pub_v2x_goal_;

  // === Callbacks ===
  void onAutowareTrajectory(const TrajectoryMsg::ConstSharedPtr msg);
  void onAutowareTf(const TfMsg::ConstSharedPtr msg);
  void onAutowareGoal(const GoalMsg::ConstSharedPtr msg);
};
}  // namespace rsu_autoware_bridge