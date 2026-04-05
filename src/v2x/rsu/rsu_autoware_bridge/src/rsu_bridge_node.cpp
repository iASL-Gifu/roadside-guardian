#include "rsu_autoware_bridge/rsu_bridge_node.hpp"
#include <limits>
#include <cmath>

namespace rsu_autoware_bridge
{

RsuBridgeNode::RsuBridgeNode(const rclcpp::NodeOptions & options) 
: Node("rsu_bridge_node", options), planning_override_enabled_(false)
{
  // Parameters
  param_score_threshold_ = this->declare_parameter("validation.score_threshold", 2.5);
  param_weight_lat_ = this->declare_parameter("validation.weight_lateral", 1.1);
  param_weight_yaw_ = this->declare_parameter("validation.weight_yaw", 11.1);
  param_weight_vel_ = this->declare_parameter("validation.weight_velocity", 0.6);

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
  auto qos_subscriber = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  auto qos_tf = rclcpp::QoS(rclcpp::KeepLast(100)).reliable().durability_volatile();

  // Subscribers / Publishers
  sub_v2x_traj_ = this->create_subscription<AutoTrajectoryMsg>(
    "/v2x/vehicle/trajectory", qos,
    std::bind(&RsuBridgeNode::onV2XTrajectory, this, std::placeholders::_1));

  sub_v2x_tf_ = this->create_subscription<TfMsg>(
    "/v2x/vehicle/tf", qos_tf,
    std::bind(&RsuBridgeNode::onV2XTf, this, std::placeholders::_1));

  sub_v2x_goal_ = this->create_subscription<GoalMsg>(
    "/v2x/vehicle/goal", qos,
    std::bind(&RsuBridgeNode::onV2XGoal, this, std::placeholders::_1));

  sub_planning_override_ = this->create_subscription<std_msgs::msg::Bool>(
    "/v2x/common/planning_override/enable", qos,
    std::bind(&RsuBridgeNode::onPlanningOverride, this, std::placeholders::_1));

  pub_rsu_traj_ = this->create_publisher<UniverseTrajectoryMsg>("/rsu/vehicle/trajectory", qos);
  pub_rsu_tf_ = this->create_publisher<TfMsg>("/rsu/vehicle/tf", qos_tf);
  pub_rsu_goal_ = this->create_publisher<GoalMsg>("/rsu/vehicle/goal", qos);
  
  sub_rsu_corrected_traj_ = this->create_subscription<UniverseTrajectoryMsg>(
    "/planning/trajectory", qos_subscriber, 
    std::bind(&RsuBridgeNode::onRsuCorrectedTrajectory, this, std::placeholders::_1));

  pub_v2x_corrected_traj_ = this->create_publisher<AutoTrajectoryMsg>("/v2x/rsu/corrected_trajectory", qos);

  RCLCPP_INFO(this->get_logger(), "RSU Bridge Node running. Monitoring mode (Silent unless deviation detected).");
}

// Conversion functions
UniverseTrajectoryMsg RsuBridgeNode::convertToUniverse(const AutoTrajectoryMsg& msg) {
  UniverseTrajectoryMsg out_msg; out_msg.header = msg.header;
  for (const auto& p : msg.points) {
    autoware_planning_msgs::msg::TrajectoryPoint up;
    up.time_from_start = p.time_from_start;
    up.pose = p.pose;
    up.longitudinal_velocity_mps = p.longitudinal_velocity_mps;
    up.lateral_velocity_mps = p.lateral_velocity_mps;
    up.acceleration_mps2 = p.acceleration_mps2;
    up.heading_rate_rps = p.heading_rate_rps;
    up.front_wheel_angle_rad = p.front_wheel_angle_rad;
    up.rear_wheel_angle_rad = p.rear_wheel_angle_rad;
    out_msg.points.push_back(up);
  }
  return out_msg;
}
AutoTrajectoryMsg RsuBridgeNode::convertToAuto(const UniverseTrajectoryMsg& msg) {
  AutoTrajectoryMsg out_msg; out_msg.header = msg.header;
  for (const auto& p : msg.points) {
    autoware_auto_planning_msgs::msg::TrajectoryPoint ap;
    ap.time_from_start = p.time_from_start;
    ap.pose = p.pose;
    ap.longitudinal_velocity_mps = p.longitudinal_velocity_mps;
    ap.lateral_velocity_mps = p.lateral_velocity_mps;
    ap.acceleration_mps2 = p.acceleration_mps2;
    ap.heading_rate_rps = p.heading_rate_rps;
    ap.front_wheel_angle_rad = p.front_wheel_angle_rad;
    ap.rear_wheel_angle_rad = p.rear_wheel_angle_rad;
    out_msg.points.push_back(ap);
  }
  return out_msg;
}

// === Callbacks ===

void RsuBridgeNode::onPlanningOverride(const std_msgs::msg::Bool::ConstSharedPtr msg)
{
  planning_override_enabled_ = msg->data;
  if (planning_override_enabled_) {
     RCLCPP_WARN(this->get_logger(), "Override ON: RSU will enforce its trajectory.");
  } else {
     RCLCPP_INFO(this->get_logger(), "Override OFF: Monitoring vehicle trajectory.");
  }
}

// Note: Only save when receiving RSU trajectory
void RsuBridgeNode::onRsuCorrectedTrajectory(const UniverseTrajectoryMsg::ConstSharedPtr msg)
{
  // Save as the latest reference trajectory (for comparison and intervention)
  latest_rsu_traj_ = msg;
}

// Note: Compare, validate, and intervene when receiving vehicle trajectory
void RsuBridgeNode::onV2XTrajectory(const AutoTrajectoryMsg::ConstSharedPtr msg)
{
  // 1. Forward to RSU internally (for visualization)
  auto uni_msg = convertToUniverse(*msg);
  pub_rsu_traj_->publish(uni_msg);

  // 2. Validation (deviation check)
  // Returns True if normal, False if abnormal (deviation or override)
  if (!checkIfTrajectoryIsValid(msg)) 
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
      "!!! DEVIATION DETECTED !!! Sending Corrected Trajectory from RSU.");
    
    // 3. Publish only on abnormality: Convert RSU reference trajectory to Auto type and send
    if (latest_rsu_traj_) {
        auto corrected_msg = convertToAuto(*latest_rsu_traj_);
        
        // For latency measurement, keep the original message header.
        // This allows the vehicle to calculate total loop latency (Vehicle->RSU->Vehicle)
        // by calculating (ReceiveTime - Stamp).
        corrected_msg.header = msg->header;

        pub_v2x_corrected_traj_->publish(corrected_msg);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Cannot intervene: RSU trajectory is not available yet.");
    }
  }
}

// Validation logic
bool RsuBridgeNode::checkIfTrajectoryIsValid(const AutoTrajectoryMsg::ConstSharedPtr msg)
{
  // If override is ON, forcefully return false for intervention
  if (planning_override_enabled_) return false;

  // Return true if there's no reference trajectory to compare against
  if (!latest_rsu_traj_) return true;
  if (latest_rsu_traj_->points.empty() || msg->points.empty()) return true;

  // Calculate score
  auto vehicle_traj_uni = convertToUniverse(*msg);
  double score = calculateTrajectoryScore(vehicle_traj_uni, *latest_rsu_traj_);

  // Check if score exceeds threshold (currently monitoring only)
  if (score > param_score_threshold_) {
    RCLCPP_WARN(this->get_logger(), "Validation Score: %.2f (Threshold: %.2f) -> DEVIATION DETECTED (Monitoring Only)", score, param_score_threshold_);
    return true; 
  }

  return true;
}

// Score calculation
double RsuBridgeNode::calculateTrajectoryScore(
  const UniverseTrajectoryMsg& vehicle_traj, 
  const UniverseTrajectoryMsg& rsu_traj)
{
  double total_error = 0.0;
  int compared_points = 0;

  for (const auto& v_point : vehicle_traj.points) {
    double min_dist = std::numeric_limits<double>::max();
    size_t nearest_idx = 0;
    
    for (size_t i = 0; i < rsu_traj.points.size(); ++i) {
      double d = autoware::universe_utils::calcDistance2d(v_point.pose, rsu_traj.points[i].pose);
      if (d < min_dist) {
        min_dist = d;
        nearest_idx = i;
      }
    }

    if (min_dist > 5.0) continue; 

    const auto& r_point = rsu_traj.points[nearest_idx];

    double lat_error = min_dist;
    double v_yaw = tf2::getYaw(v_point.pose.orientation);
    double r_yaw = tf2::getYaw(r_point.pose.orientation);
    double yaw_error = std::abs(autoware::universe_utils::normalizeRadian(v_yaw - r_yaw));
    double vel_error = std::abs(v_point.longitudinal_velocity_mps - r_point.longitudinal_velocity_mps);

    double point_score = (lat_error * param_weight_lat_) + 
                         (yaw_error * param_weight_yaw_) + 
                         (vel_error * param_weight_vel_);
    
    total_error += point_score;
    compared_points++;
  }

  if (compared_points == 0) return 0.0;
  return total_error / compared_points;
}



void RsuBridgeNode::onV2XTf(const TfMsg::ConstSharedPtr msg) { pub_rsu_tf_->publish(*msg); }
void RsuBridgeNode::onV2XGoal(const GoalMsg::ConstSharedPtr msg) { pub_rsu_goal_->publish(*msg); }

} // namespace


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<rsu_autoware_bridge::RsuBridgeNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rsu_autoware_bridge::RsuBridgeNode)