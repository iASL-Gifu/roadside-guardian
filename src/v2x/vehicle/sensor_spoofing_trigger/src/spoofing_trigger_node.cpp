#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/odometry.hpp"

using std::placeholders::_1;

class SpoofingTriggerNode : public rclcpp::Node
{
public:
  SpoofingTriggerNode()
  : Node("spoofing_trigger_node"),
    auto_switch_enabled_(false),  // Default is Auto Mode OFF
    trigger_y_threshold_(20.0),   // Threshold
    last_spoof_state_(false)      // For state change detection
  {
    // Parameter settings (can be changed from launch like trigger_y:=15.0)
    this->declare_parameter("trigger_y", 20.0);
    trigger_y_threshold_ = this->get_parameter("trigger_y").as_double();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();

    // Auto Switch (Auto detection works only when True)
    sub_auto_switch_ = this->create_subscription<std_msgs::msg::Bool>(
      "/v2x/common/sensor_spoof/auto_switch", qos,
      std::bind(&SpoofingTriggerNode::onAutoSwitch, this, _1));

    // Vehicle position (Autoware ego position)
    sub_kinematic_state_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/localization/kinematic_state", qos,
      std::bind(&SpoofingTriggerNode::onKinematicState, this, _1));

    // Spoof signal output
    pub_spoof_signal_ = this->create_publisher<std_msgs::msg::Bool>(
      "/v2x/common/sensor_spoof/spoof", qos);

    RCLCPP_INFO(this->get_logger(), "Spoofing Trigger Node initialized. Threshold Y > %.2f", trigger_y_threshold_);
  }

private:
  void onAutoSwitch(const std_msgs::msg::Bool::SharedPtr msg)
  {
    auto_switch_enabled_ = msg->data;
    if (auto_switch_enabled_) {
      RCLCPP_INFO(this->get_logger(), "Auto Trigger ENABLED. Monitoring Y > %.2f", trigger_y_threshold_);
    } else {
      RCLCPP_INFO(this->get_logger(), "Auto Trigger DISABLED. Resetting spoof signal to FALSE.");
      // Turn off spoof immediately when auto switch is disabled (for safety)
      publishSpoofSignal(false);
    }
  }

  void onKinematicState(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Do nothing if auto switch is disabled
    if (!auto_switch_enabled_) {
      return;
    }

    // Y coordinate check: True if > 20, False if < 20
    double current_y = msg->pose.pose.position.y;
    bool should_spoof = (current_y > trigger_y_threshold_);

    // Always send current result (to prevent topic interruption)
    // Note: To reduce traffic, you could send only when state changes
    publishSpoofSignal(should_spoof);

    // Log only when state changes
    if (should_spoof != last_spoof_state_) {
      if (should_spoof) {
        RCLCPP_WARN(this->get_logger(), "!!! ENTERED SPOOF ZONE (Y=%.2f) -> SPOOF ON !!!", current_y);
      } else {
        RCLCPP_INFO(this->get_logger(), "Exited Spoof Zone (Y=%.2f) -> SPOOF OFF", current_y);
      }
      last_spoof_state_ = should_spoof;
    }
  }

  void publishSpoofSignal(bool state)
  {
    std_msgs::msg::Bool msg;
    msg.data = state;
    pub_spoof_signal_->publish(msg);
  }

  bool auto_switch_enabled_;
  double trigger_y_threshold_;
  bool last_spoof_state_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_auto_switch_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_kinematic_state_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_spoof_signal_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpoofingTriggerNode>());
  rclcpp::shutdown();
  return 0;
}