#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

class LidarSelectorNode : public rclcpp::Node
{
public:
  LidarSelectorNode()
  : Node("lidar_selector_node"), selector_flag_(false)
  {
    // --- QoS Settings (adjust according to Lidar data) ---
    // For receiving: Sensor data so Best Effort is OK
    auto sub_qos = rclcpp::SensorDataQoS();
    // For publishing: Set to Reliable (SystemDefaults) so other nodes can reliably receive it
    auto pub_qos = rclcpp::SystemDefaultsQoS();

    // --- Create Subscribers ---
    // Subscribe to flag
    flag_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/v2x/common/sensor_spoof/spoof", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        selector_flag_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Switching to: %s", selector_flag_ ? "fake_pointcloud" : "original_pointcloud");
      });

    // Subscribe to Lidar A (using sub_qos)
    lidar_a_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/sensing/lidar/concatenated/original_pointcloud", sub_qos,
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (!selector_flag_) {
          pub_->publish(*msg);
        }
      });

    // Subscribe to Lidar B (using sub_qos)
    lidar_b_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/sensing/lidar/concatenated/fake_pointcloud", sub_qos,
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (selector_flag_) {
          pub_->publish(*msg);
        }
      });

    // --- Create Publisher ---
    // Change this to pub_qos (Reliable)
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sensing/lidar/concatenated/pointcloud", pub_qos);

    RCLCPP_INFO(this->get_logger(), "Lidar Selector Node started.");
  }

private:
  // Subscriber and publisher definitions
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr flag_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_a_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_b_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  // Flag to select which one (false: A, true: B)
  bool selector_flag_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarSelectorNode>());
  rclcpp::shutdown();
  return 0;
}