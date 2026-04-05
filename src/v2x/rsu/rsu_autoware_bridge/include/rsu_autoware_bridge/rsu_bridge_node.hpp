#pragma once
#include "rclcpp/rclcpp.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "std_msgs/msg/bool.hpp"
#include "autoware/universe_utils/geometry/geometry.hpp"
#include "autoware/universe_utils/math/normalization.hpp"
#include <memory>
#include <vector>

using UniverseTrajectoryMsg = autoware_planning_msgs::msg::Trajectory;
using AutoTrajectoryMsg = autoware_auto_planning_msgs::msg::Trajectory;
using GoalMsg = geometry_msgs::msg::PoseStamped;
using TfMsg = tf2_msgs::msg::TFMessage;

namespace rsu_autoware_bridge
{

class RsuBridgeNode : public rclcpp::Node
{
public:
  explicit RsuBridgeNode(const rclcpp::NodeOptions & options);

private:
  // === Subscribers / Publishers ===
  rclcpp::Subscription<AutoTrajectoryMsg>::SharedPtr sub_v2x_traj_;
  rclcpp::Subscription<TfMsg>::SharedPtr sub_v2x_tf_;
  rclcpp::Subscription<GoalMsg>::SharedPtr sub_v2x_goal_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_planning_override_;
  rclcpp::Publisher<UniverseTrajectoryMsg>::SharedPtr pub_rsu_traj_;
  rclcpp::Publisher<TfMsg>::SharedPtr pub_rsu_tf_;
  rclcpp::Publisher<GoalMsg>::SharedPtr pub_rsu_goal_;
  rclcpp::Subscription<UniverseTrajectoryMsg>::SharedPtr sub_rsu_corrected_traj_;
  rclcpp::Publisher<AutoTrajectoryMsg>::SharedPtr pub_v2x_corrected_traj_;

  // === Member variables ===
  bool planning_override_enabled_;
  UniverseTrajectoryMsg::ConstSharedPtr latest_rsu_traj_; // Latest RSU reference trajectory

  // === Parameters (Weights and thresholds) ===
  double param_score_threshold_;
  double param_weight_lat_;
  double param_weight_yaw_;
  double param_weight_vel_;

  // === Callbacks ===
  void onV2XTrajectory(const AutoTrajectoryMsg::ConstSharedPtr msg);
  void onV2XTf(const TfMsg::ConstSharedPtr msg);
  void onV2XGoal(const GoalMsg::ConstSharedPtr msg);
  void onRsuCorrectedTrajectory(const UniverseTrajectoryMsg::ConstSharedPtr msg);
  void onPlanningOverride(const std_msgs::msg::Bool::ConstSharedPtr msg);

  // === Helpers ===
  UniverseTrajectoryMsg convertToUniverse(const AutoTrajectoryMsg& msg);
  AutoTrajectoryMsg convertToAuto(const UniverseTrajectoryMsg& msg);

  // === Validation Logic ===
  bool checkIfTrajectoryIsValid(const AutoTrajectoryMsg::ConstSharedPtr msg);
  double calculateTrajectoryScore(const UniverseTrajectoryMsg& vehicle_traj, const UniverseTrajectoryMsg& rsu_traj); // Score calculation
};
}  // namespace rsu_autoware_bridge