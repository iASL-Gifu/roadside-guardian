#include "rsu_autoware_bridge/vehicle_bridge_node.hpp"

namespace rsu_autoware_bridge
{
VehicleBridgeNode::VehicleBridgeNode(const rclcpp::NodeOptions & options) : Node("vehicle_bridge_node", options)
{
  // Volatile setting (to match Autoware)
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
  // Specific QoS for high frequency /tf topic
  auto qos_tf = rclcpp::QoS(rclcpp::KeepLast(100)).reliable().durability_volatile();

  // === Subscribers from Autoware ===
  sub_autoware_traj_ = this->create_subscription<TrajectoryMsg>(
    "/planning/scenario_planning/vehicle_trajectory", qos,
    std::bind(&VehicleBridgeNode::onAutowareTrajectory, this, std::placeholders::_1));
  sub_autoware_tf_ = this->create_subscription<TfMsg>(
    "/tf", qos_tf,
    std::bind(&VehicleBridgeNode::onAutowareTf, this, std::placeholders::_1));
  // Goal (RViz, etc.)
  sub_autoware_goal_ = this->create_subscription<GoalMsg>(
    "/planning/mission_planning/goal", qos,
    std::bind(&VehicleBridgeNode::onAutowareGoal, this, std::placeholders::_1));

  // === Publishers to RSU (V2X) ===
  pub_v2x_traj_ = this->create_publisher<TrajectoryMsg>("/v2x/vehicle/trajectory", qos);
  pub_v2x_tf_ = this->create_publisher<TfMsg>("/v2x/vehicle/tf", qos_tf);
  pub_v2x_goal_ = this->create_publisher<GoalMsg>("/v2x/vehicle/goal", qos);
}

// Forward Autoware data directly to V2X topics
void VehicleBridgeNode::onAutowareTrajectory(const TrajectoryMsg::ConstSharedPtr msg)
{
  // Throttle log output
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Received Trajectory. Forwarding...");
  pub_v2x_traj_->publish(*msg);
}

void VehicleBridgeNode::onAutowareTf(const TfMsg::ConstSharedPtr msg)
{
  pub_v2x_tf_->publish(*msg);
}

void VehicleBridgeNode::onAutowareGoal(const GoalMsg::ConstSharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received Goal. Forwarding...");
  pub_v2x_goal_->publish(*msg);
}

}  // namespace rsu_autoware_bridge

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<rsu_autoware_bridge::VehicleBridgeNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rsu_autoware_bridge::VehicleBridgeNode)