#include "rsu_autoware_bridge/vehicle_trajectory_arbiter_node.hpp"

namespace rsu_autoware_bridge
{
TrajectoryArbiterNode::TrajectoryArbiterNode(const rclcpp::NodeOptions & options) : Node("vehicle_trajectory_arbiter_node", options)
{
  using namespace std::chrono_literals;

  // Use Volatile QoS for all topics
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();

  // Subscribe to inputs from Planning Validator
  sub_planning_traj_ = this->create_subscription<TrajectoryMsg>(
    "/planning/scenario_planning/vehicle_trajectory", qos,
    std::bind(&TrajectoryArbiterNode::onPlanningTrajectory, this, std::placeholders::_1));

  // Subscribe to intervention trajectory from RSU
  sub_v2x_intervention_ = this->create_subscription<TrajectoryMsg>(
    "/v2x/rsu/corrected_trajectory", qos,
    std::bind(&TrajectoryArbiterNode::onV2XIntervention, this, std::placeholders::_1));

  // Publish final trajectory to control system
  pub_final_traj_ = this->create_publisher<TrajectoryMsg>("/planning/scenario_planning/trajectory", qos);

  // Planning override status
  pub_override_ = this->create_publisher<std_msgs::msg::Bool>("/v2x/common/planning_override/working", qos);

  // Timer for arbitration logic (approx 10Hz to match Planning)
  arbitration_timer_ = rclcpp::create_timer(this, this->get_clock(), 100ms, std::bind(&TrajectoryArbiterNode::onArbitrationTimer, this));

  RCLCPP_INFO(this->get_logger(), "Trajectory Arbiter Node is running.");
}

void TrajectoryArbiterNode::onPlanningTrajectory(const TrajectoryMsg::ConstSharedPtr msg)
{
  latest_planning_traj_ = msg;
}

void TrajectoryArbiterNode::onV2XIntervention(const TrajectoryMsg::ConstSharedPtr msg)
{
  latest_v2x_traj_ = msg;
}

// Check if intervention trajectory is valid (within timeout)
bool TrajectoryArbiterNode::isInterventionValid()
{
  if (!latest_v2x_traj_) return false;

  const auto time_diff = this->get_clock()->now() - latest_v2x_traj_->header.stamp;
  
  if (time_diff.seconds() < INTERVENTION_TIMEOUT_SEC)
  {
      return true;
  }
  return false;
}


void TrajectoryArbiterNode::onArbitrationTimer()
{
  // 1. Do nothing if no data is available
  if (!latest_planning_traj_ && !latest_v2x_traj_) return;

  // 2. Arbitration logic
  if (isInterventionValid())
  {
    // Prioritize V2X intervention if valid
    pub_final_traj_->publish(*latest_v2x_traj_);
    std_msgs::msg::Bool msg;
    msg.data = true;
    pub_override_->publish(msg);
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "V2X INTERVENTION ACTIVE!");
  }
  else if (latest_planning_traj_)
  {
    // Use Planning trajectory if V2X intervention is invalid
    pub_final_traj_->publish(*latest_planning_traj_);
    std_msgs::msg::Bool msg;
    msg.data = false;
    pub_override_->publish(msg);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Using Planning Trajectory.");
  }
  // 3. Invalidate the latest V2X message if intervention times out
  if (!isInterventionValid() && latest_v2x_traj_) {
    latest_v2x_traj_ = nullptr;
    RCLCPP_INFO_ONCE(this->get_logger(), "V2X Intervention timed out.");
  }
}

}  // namespace rsu_autoware_bridge

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<rsu_autoware_bridge::TrajectoryArbiterNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rsu_autoware_bridge::TrajectoryArbiterNode)