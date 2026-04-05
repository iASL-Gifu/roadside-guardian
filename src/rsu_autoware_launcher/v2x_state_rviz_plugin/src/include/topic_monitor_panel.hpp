#ifndef TOPIC_MONITOR_PANEL_HPP_
#define TOPIC_MONITOR_PANEL_HPP_

#include "custom_icon_label.hpp"
#include "custom_container.hpp"
#include "custom_button.hpp"
#include <QLabel>
#include <QVBoxLayout>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <vector>
#include <string>
#include <memory>
#include <std_msgs/msg/float32.hpp>
#include <QLineEdit>
#include <std_msgs/msg/string.hpp>

namespace v2x_rviz_plugins
{

class TopicMonitorPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit TopicMonitorPanel(QWidget * parent = nullptr);
  void onInitialize() override;

protected:
  // 各トピックの管理情報
  struct TopicInfo {
    std::string topic_name;       // 監視するトピック名
    std::string display_name;     // 表示名
    std::string message_type;     // 型（今回は汎用サブスクリプションを使用想定、または固定）
    
    rclcpp::Time last_callback_time; // 最終受信時刻
    CustomIconLabel * icon_label;    // アイコンウィジェット
    QLabel * text_label;             // テキストウィジェット
    rclcpp::SubscriptionBase::SharedPtr sub; // サブスクライバー
  };

  // UIに行を追加するヘルパー関数
  void addTopicRow(std::shared_ptr<TopicInfo> info, QVBoxLayout* layout);

  // 監視対象リスト
  std::vector<std::shared_ptr<TopicInfo>> topics_;

  // --- Spoof制御用 ---
  void createSpoofControl(QVBoxLayout* layout);
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr spoof_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr spoof_sub_;
  // UIパーツ
  CustomIconLabel * spoof_icon_;
  QLabel * spoof_status_label_;
  CustomElevatedButton * spoof_button_;
  // データ保持用
  bool latest_spoof_value_{false};
  rclcpp::Time last_spoof_rcv_time_;
  bool spoof_cmd_{false};

  // --- Auto Switch制御用 ---
  void createAutoSwitchControl(QVBoxLayout* layout);
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr auto_switch_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr auto_switch_sub_;
  // UIパーツ
  CustomIconLabel * auto_switch_icon_;
  QLabel * auto_switch_status_label_;
  CustomElevatedButton * auto_switch_button_;
  // データ保持用
  bool latest_auto_switch_value_{false};
  rclcpp::Time last_auto_switch_rcv_time_;
  bool auto_switch_cmd_{false};
  
  // --- 介入制御用 ---
  void createOverrideControl(QVBoxLayout* layout);
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr override_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr override_sub_;
  // UIパーツ
  CustomIconLabel * override_icon_;
  QLabel * override_status_label_;
  CustomElevatedButton * override_button_;
  // データ保持用
  bool latest_override_value_{false};
  rclcpp::Time last_override_rcv_time_;
  bool override_cmd_{false};

  // --- Delay監視用 ---
  void createDelayMonitor(QVBoxLayout* layout);
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr delay_sub_;
  CustomIconLabel * delay_icon_; // ステータスアイコン
  QLabel * delay_label_;         // 値表示用ラベル
  float latest_delay_value_{0.0f};
  rclcpp::Time last_delay_rcv_time_;

  // --- Description表示用 ---
  void createDescriptionControl(QVBoxLayout* layout);
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr description_pub_;
  QLineEdit * description_edit_;       // テキスト入力ボックス
  CustomElevatedButton * description_button_; // 送信ボタン

  // 共通
  rclcpp::Node::SharedPtr raw_node_;
  QTimer * update_timer_;

protected Q_SLOTS:
  void onTimer();
  void onSpoofButton();
  void onAutoSwitchButton();
  void onOverrideButton();
  void onDescriptionButton();
};

} // namespace v2x_rviz_plugins

#endif