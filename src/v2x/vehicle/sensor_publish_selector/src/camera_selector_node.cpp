#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include <sensor_msgs/msg/image.hpp>

class CameraSelectorNode : public rclcpp::Node
{
public:
  CameraSelectorNode()
  : Node("camera_selector_node"), selector_flag_(false)
  {
    // --- QoS Settings (adjust according to Camera data) ---
    auto sub_qos = rclcpp::SensorDataQoS();      // For receiving (Best Effort)
    auto pub_qos = rclcpp::SystemDefaultsQoS();  // For publishing (Reliable)

    // --- Create Subscribers ---
    // Subscribe to flag
    flag_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/v2x/common/sensor_spoof/spoof", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        selector_flag_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Switching to: %s", selector_flag_ ? "fake_image_raw" : "original_image_raw");
      });

    // Subscribe to Camera A
    camera_a_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/sensing/camera/traffic_light/original_image_raw", sub_qos,
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!selector_flag_) {
          pub_->publish(*msg);
        }
      });

    // Subscribe to Camera B
    camera_b_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/sensing/camera/traffic_light/fake_image_raw", sub_qos,
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        if (selector_flag_) {
          pub_->publish(*msg);
        }
      });

    // --- Create Publisher ---
    pub_ = this->create_publisher<sensor_msgs::msg::Image>("/sensing/camera/traffic_light/image_raw", pub_qos);

    RCLCPP_INFO(this->get_logger(), "Camera Selector Node started.");
  }

private:
  // Subscriber and publisher definitions
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr flag_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_a_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_b_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;

  // Flag to select which one (false: A, true: B)
  bool selector_flag_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraSelectorNode>());
  rclcpp::shutdown();
  return 0;
}