#pragma once
#include "rclcpp/rclcpp.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include <std_msgs/msg/bool.hpp>

using TrajectoryMsg = autoware_auto_planning_msgs::msg::Trajectory;

namespace rsu_autoware_bridge
{
class TrajectoryArbiterNode : public rclcpp::Node
{
public:
  explicit TrajectoryArbiterNode(const rclcpp::NodeOptions & options);

private:
  // Parameter: Time threshold to consider V2X intervention valid (0.5s)
  const double INTERVENTION_TIMEOUT_SEC = 0.5;

  rclcpp::Subscription<TrajectoryMsg>::SharedPtr sub_planning_traj_;
  rclcpp::Subscription<TrajectoryMsg>::SharedPtr sub_v2x_intervention_;
  rclcpp::Publisher<TrajectoryMsg>::SharedPtr pub_final_traj_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_override_;

  TrajectoryMsg::ConstSharedPtr latest_planning_traj_ = nullptr;
  TrajectoryMsg::ConstSharedPtr latest_v2x_traj_ = nullptr;

  rclcpp::TimerBase::SharedPtr arbitration_timer_;

  void onPlanningTrajectory(const TrajectoryMsg::ConstSharedPtr msg);
  void onV2XIntervention(const TrajectoryMsg::ConstSharedPtr msg);
  void onArbitrationTimer();

  bool isInterventionValid();
};
}  // namespace rsu_autoware_bridge