#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"

using std::placeholders::_1;

class V2XLatencyMonitor : public rclcpp::Node
{
public:
  V2XLatencyMonitor()
  : Node("v2x_latency_monitor")
  {
    // Declare parameters (set default values)
    this->declare_parameter("input_topic", "/v2x/rsu/corrected_trajectory"); // Topic coming back from RSU
    this->declare_parameter("output_topic", "/v2x/common/delay");          // Delay topic

    std::string input_topic_name = this->get_parameter("input_topic").as_string();
    std::string output_topic_name = this->get_parameter("output_topic").as_string();

    // Subscriber (Trajectory from RSU)
    // Use System Default QoS to stabilize behavior during packet loss
    rclcpp::QoS qos_settings(1);
    qos_settings.reliable();
    qos_settings.durability_volatile();

    sub_trajectory_ = this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
      input_topic_name,
      qos_settings,
      std::bind(&V2XLatencyMonitor::topic_callback, this, _1));

    // Publisher (Delay time [ms])
    pub_delay_ = this->create_publisher<std_msgs::msg::Float32>(output_topic_name, 10);

    RCLCPP_INFO(this->get_logger(), "Monitoring latency on topic: %s", input_topic_name.c_str());
  }

private:
  void topic_callback(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg)
  {
    // Get current time
    rclcpp::Time now = this->now();
    
    // Get message header time
    rclcpp::Time msg_time = msg->header.stamp;

    // Calculate delay (Current time - Message generation time)
    // If RSU copies the "request source time" and returns it, it is "Round-trip delay"
    // If RSU puts the "reply generation time", it is "One-way delay + Clock error"
    double latency_seconds = (now - msg_time).seconds();

    // Convert to milliseconds
    float latency_ms = static_cast<float>(latency_seconds * 1000.0);

    // Filter abnormal values (e.g., future time) (Clock drift countermeasure)
    if (latency_ms < 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
        "Negative latency detected (%.2f ms). Check clock synchronization or RSU timestamp logic.", latency_ms);
      // Output as is to see the trend even if it's a negative value
    }

    // Publish
    std_msgs::msg::Float32 delay_msg;
    delay_msg.data = latency_ms;
    pub_delay_->publish(delay_msg);

    RCLCPP_DEBUG(this->get_logger(), "Latency: %.2f ms", latency_ms);
  }

  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr sub_trajectory_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_delay_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<V2XLatencyMonitor>());
  rclcpp::shutdown();
  return 0;
}