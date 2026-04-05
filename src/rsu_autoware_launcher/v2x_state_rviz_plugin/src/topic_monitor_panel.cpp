#include "include/topic_monitor_panel.hpp"
#include "include/material_colors.hpp"
#include <rviz_common/display_context.hpp>
#include <rclcpp/serialization.hpp>
#include <QFrame>

namespace v2x_rviz_plugins
{

TopicMonitorPanel::TopicMonitorPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  auto * main_layout = new QVBoxLayout;
  auto * container = new CustomContainer(this);
  
  main_layout->setSpacing(5);
  main_layout->setContentsMargins(10, 10, 10, 10);
  container->getLayout()->addLayout(main_layout, 0, 0);

  // 1. 汎用トピック監視リスト
  std::vector<std::pair<std::string, std::string>> target_topics = {
    {"/localization/kinematic_state", "(Input) /localization/kinematic_state"},
    {"/planning/mission_planning/goal", "(Input) /planning/mission_planning/goal"},
    {"/perception/object_recognition/objects", "(Input) /perception/object_recognition/objects"},
    {"/planning/trajectory", "(Output) /planning/trajectory"},
    // 必要に応じて追加
  };

  for (const auto& target : target_topics) {
    auto info = std::make_shared<TopicInfo>();
    info->topic_name = target.first;
    info->display_name = target.second;
    // 【重要】RCL_ROS_TIMEで初期化して計算エラーを防止
    info->last_callback_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
    
    addTopicRow(info, main_layout);
    topics_.push_back(info);
  }

  // 2. 区切り線
  auto * line = new QFrame();
  line->setFrameShape(QFrame::HLine);
  line->setFrameShadow(QFrame::Sunken);
  line->setStyleSheet("background-color: #555;"); // 少し見えるように色付け
  main_layout->addSpacing(10);
  main_layout->addWidget(line);
  main_layout->addSpacing(10);

  // 3. Spoofコントロール作成
  createSpoofControl(main_layout);
  createAutoSwitchControl(main_layout);
  createOverrideControl(main_layout);

  // 4. Delay監視作成
  createDelayMonitor(main_layout);

  // 5. Description監視作成
  createDescriptionControl(main_layout);

  // パネル設定
  auto * panel_layout = new QVBoxLayout;
  panel_layout->addWidget(container);
  panel_layout->addStretch();
  setLayout(panel_layout);

  // タイマー
  update_timer_ = new QTimer(this);
  connect(update_timer_, &QTimer::timeout, this, &TopicMonitorPanel::onTimer);
  update_timer_->start(500); // UI更新頻度 (0.5秒)
}

void TopicMonitorPanel::addTopicRow(std::shared_ptr<TopicInfo> info, QVBoxLayout* layout)
{
  auto * h_layout = new QHBoxLayout;
  info->icon_label = new CustomIconLabel(QColor("#B0B0B0")); // Gray
  info->text_label = new QLabel(QString::fromStdString(info->display_name));
  info->text_label->setStyleSheet("color: white; font-weight: bold;");

  h_layout->addWidget(info->icon_label);
  h_layout->addWidget(info->text_label);
  h_layout->addStretch();
  layout->addLayout(h_layout);
}

void TopicMonitorPanel::createSpoofControl(QVBoxLayout* layout)
{
  auto * h_layout = new QHBoxLayout;

  // アイコン (通信状態用)
  spoof_icon_ = new CustomIconLabel(QColor("#B0B0B0"));

  // ラベル (値表示用)
  spoof_status_label_ = new QLabel("Sensor Spoof: WAITING");
  spoof_status_label_->setStyleSheet("color: gray; font-weight: bold;");

  // ボタン (操作用)
  spoof_button_ = new CustomElevatedButton("Turn ON");
  spoof_button_->setCheckable(false);
  // ボタンクリック時の動作を登録
  connect(spoof_button_, &CustomElevatedButton::clicked, this, &TopicMonitorPanel::onSpoofButton);

  h_layout->addWidget(spoof_icon_);
  h_layout->addWidget(spoof_status_label_);
  h_layout->addStretch();
  h_layout->addWidget(spoof_button_);

  layout->addLayout(h_layout);

  // 時刻初期化
  last_spoof_rcv_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

void TopicMonitorPanel::createAutoSwitchControl(QVBoxLayout* layout)
{
  auto * h_layout = new QHBoxLayout;

  // アイコン (通信状態用)
  auto_switch_icon_ = new CustomIconLabel(QColor("#B0B0B0"));

  // ラベル (値表示用)
  auto_switch_status_label_ = new QLabel("Auto Spoof Switch: WAITING");
  auto_switch_status_label_->setStyleSheet("color: gray; font-weight: bold;");

  // ボタン (操作用)
  auto_switch_button_ = new CustomElevatedButton("Turn ON");
  auto_switch_button_->setCheckable(false);
  // ボタンクリック時の動作を登録
  connect(auto_switch_button_, &CustomElevatedButton::clicked, this, &TopicMonitorPanel::onAutoSwitchButton);

  h_layout->addWidget(auto_switch_icon_);
  h_layout->addWidget(auto_switch_status_label_);
  h_layout->addStretch();
  h_layout->addWidget(auto_switch_button_);

  layout->addLayout(h_layout);

  // 時刻初期化
  last_auto_switch_rcv_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

void TopicMonitorPanel::createOverrideControl(QVBoxLayout* layout)
{
  auto * h_layout = new QHBoxLayout;

  // アイコン (通信状態用)
  override_icon_ = new CustomIconLabel(QColor("#B0B0B0"));

  // ラベル (値表示用)
  override_status_label_ = new QLabel("Planning Override: WAITING");
  override_status_label_->setStyleSheet("color: gray; font-weight: bold;");

  // ボタン (操作用)
  override_button_ = new CustomElevatedButton("Turn ON");
  override_button_->setCheckable(false);
  // ボタンクリック時の動作を登録
  connect(override_button_, &CustomElevatedButton::clicked, this, &TopicMonitorPanel::onOverrideButton);

  h_layout->addWidget(override_icon_);
  h_layout->addWidget(override_status_label_);
  h_layout->addStretch();
  h_layout->addWidget(override_button_);

  layout->addLayout(h_layout);

  // 時刻初期化
  last_override_rcv_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

void TopicMonitorPanel::createDelayMonitor(QVBoxLayout* layout)
{
  auto * h_layout = new QHBoxLayout;

  // アイコン
  delay_icon_ = new CustomIconLabel(QColor("#B0B0B0"));
  
  // ラベル (初期値)
  delay_label_ = new QLabel("Delay: WAITING");
  delay_label_->setStyleSheet("color: gray; font-weight: bold;");

  h_layout->addWidget(delay_icon_);
  h_layout->addWidget(delay_label_);
  h_layout->addStretch(); // 左寄せ

  layout->addLayout(h_layout);
  
  last_delay_rcv_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

void TopicMonitorPanel::createDescriptionControl(QVBoxLayout* layout)
{
  auto * h_layout = new QHBoxLayout;

  // ラベル (任意)
  auto * label = new QLabel("Description:");
  label->setStyleSheet("color: white; font-weight: bold;");
  
  // テキストボックス
  description_edit_ = new QLineEdit();
  description_edit_->setPlaceholderText("Enter text here..."); // プレースホルダー
  description_edit_->setStyleSheet("background-color: white; color: black; border-radius: 4px; padding: 2px;");

  // 送信ボタン
  description_button_ = new CustomElevatedButton("Send");
  description_button_->setCheckable(false);
  // ボタンクリック時のイベント接続
  connect(description_button_, &CustomElevatedButton::clicked, this, &TopicMonitorPanel::onDescriptionButton);

  h_layout->addWidget(label);
  h_layout->addWidget(description_edit_);
  h_layout->addWidget(description_button_);

  layout->addLayout(h_layout);
}

void TopicMonitorPanel::onInitialize()
{
  raw_node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  // --- Spoof用のPub/Sub設定 ---
  // Publisher: ボタンで値を送る用
  spoof_pub_ = raw_node_->create_publisher<std_msgs::msg::Bool>(
    "/v2x/common/sensor_spoof/spoof", rclcpp::QoS(1).reliable());

  // Subscriber: 現在の値を監視・可視化する用
  spoof_sub_ = raw_node_->create_subscription<std_msgs::msg::Bool>(
    "/v2x/common/sensor_spoof/spoof", rclcpp::QoS(1),
    [this](const std_msgs::msg::Bool::ConstSharedPtr msg) {
      latest_spoof_value_ = msg->data;
      last_spoof_rcv_time_ = raw_node_->now();
    });

  // --- Auto Switch用のPub/Sub設定 ---
  // Publisher: ボタンで値を送る用
  auto_switch_pub_ = raw_node_->create_publisher<std_msgs::msg::Bool>(
    "/v2x/common/sensor_spoof/auto_switch", rclcpp::QoS(1).reliable());

  // Subscriber: 現在の値を監視・可視化する用
  auto_switch_sub_ = raw_node_->create_subscription<std_msgs::msg::Bool>(
    "/v2x/common/sensor_spoof/auto_switch", rclcpp::QoS(1),
    [this](const std_msgs::msg::Bool::ConstSharedPtr msg) {
      latest_auto_switch_value_ = msg->data;
      last_auto_switch_rcv_time_ = raw_node_->now();
    });

  override_pub_ = raw_node_->create_publisher<std_msgs::msg::Bool>(
    "/v2x/common/planning_override/enable", rclcpp::QoS(1).reliable());

  // Subscriber: 現在の値を監視・可視化する用
  override_sub_ = raw_node_->create_subscription<std_msgs::msg::Bool>(
    "/v2x/common/planning_override/enable", rclcpp::QoS(1),
    [this](const std_msgs::msg::Bool::ConstSharedPtr msg) {
      latest_override_value_ = msg->data;
      last_override_rcv_time_ = raw_node_->now();
    });

  // Delay用のSubscriber
  delay_sub_ = raw_node_->create_subscription<std_msgs::msg::Float32>(
    "/v2x/common/delay", rclcpp::QoS(1),
    [this](const std_msgs::msg::Float32::ConstSharedPtr msg) {
      latest_delay_value_ = msg->data;
      last_delay_rcv_time_ = raw_node_->now();
    });

  // --- Description用のPub ---
  description_pub_ = raw_node_->create_publisher<std_msgs::msg::String>(
    "/v2x/common/description", rclcpp::QoS(1).reliable());
}

void TopicMonitorPanel::onSpoofButton()
{
  // 現在のコマンド状態を反転
  spoof_cmd_ = !spoof_cmd_;
  
  // 即時反映のため一度Publish (onTimerでも呼ばれるが応答性向上のため)
  if (spoof_pub_) {
    std_msgs::msg::Bool msg;
    msg.data = spoof_cmd_;
    spoof_pub_->publish(msg);
  }
}

void TopicMonitorPanel::onAutoSwitchButton()
{
  // 現在のコマンド状態を反転
  auto_switch_cmd_ = !auto_switch_cmd_;
  
  // 即時反映のため一度Publish (onTimerでも呼ばれるが応答性向上のため)
  if (auto_switch_pub_) {
    std_msgs::msg::Bool msg;
    msg.data = auto_switch_cmd_;
    auto_switch_pub_->publish(msg);
  }
}

void TopicMonitorPanel::onOverrideButton()
{
  // 現在のコマンド状態を反転
  override_cmd_ = !override_cmd_;
  
  if (override_pub_) {
    std_msgs::msg::Bool msg;
    msg.data = override_cmd_;
    override_pub_->publish(msg);
  }
}

void TopicMonitorPanel::onDescriptionButton()
{
  if (!description_pub_) return;

  // テキストボックスから文字列を取得
  QString text = description_edit_->text();
  
  if (text.isEmpty()) {
    // 空の場合は送らない、または警告ログを出すなど
    return;
  }

  // メッセージ作成と送信
  std_msgs::msg::String msg;
  msg.data = text.toStdString();
  description_pub_->publish(msg);

  // 送信後にテキストボックスをクリアしたい場合は以下をコメントアウト解除
  // description_edit_->clear();
}

void TopicMonitorPanel::onTimer()
{
  if (!raw_node_) return;
  rclcpp::Time now = raw_node_->now();
  
  // ==========================================
  // 1. 汎用トピック監視ロジック
  // ==========================================
  // (自動サブスクライブ機能を含む)
  bool need_scan = false;
  for (const auto & info : topics_) {
    if (!info->sub) { need_scan = true; break; }
  }
  std::map<std::string, std::vector<std::string>> topic_list;
  if (need_scan) topic_list = raw_node_->get_topic_names_and_types();

  for (auto & info : topics_) {
    size_t pub_count = raw_node_->count_publishers(info->topic_name);
    
    // 未接続なら接続試行
    if (!info->sub && pub_count > 0 && topic_list.count(info->topic_name)) {
      auto types = topic_list.at(info->topic_name);
      if (!types.empty()) {
        info->sub = raw_node_->create_generic_subscription(
          info->topic_name, types[0], rclcpp::SensorDataQoS(),
          [info, this](std::shared_ptr<rclcpp::SerializedMessage>) {
            info->last_callback_time = raw_node_->now();
          });
      }
    }

    double elapsed = (now - info->last_callback_time).seconds();
    
    if (pub_count == 0) {
      info->icon_label->updateStyle(IconState::Danger, QColor("#F08B8B")); // Red
      info->text_label->setText(QString::fromStdString(info->display_name + " (No Pub)"));
    } else if (elapsed > 2.0) {
      info->icon_label->updateStyle(IconState::Pending, QColor("#EEF08B")); // Yellow
      info->text_label->setText(QString::fromStdString(info->display_name + " (Stalled)"));
    } else {
      info->icon_label->updateStyle(IconState::Active, QColor("#8DF08B")); // Green
      info->text_label->setText(QString::fromStdString(info->display_name));
    }
  }

  // ==========================================
  // 2. Spoof制御パネル更新ロジック
  // ==========================================
  // 保持しているコマンド値を継続的に送信
  if (spoof_pub_) {
    std_msgs::msg::Bool msg;
    msg.data = spoof_cmd_;
    spoof_pub_->publish(msg);
  }
  if (auto_switch_pub_) {
    std_msgs::msg::Bool msg;
    msg.data = auto_switch_cmd_;
    auto_switch_pub_->publish(msg);
  }
  if (override_pub_) {
    std_msgs::msg::Bool msg;
    msg.data = override_cmd_;
    override_pub_->publish(msg);
  }

  // UI更新
  double spoof_elapsed = (now - last_spoof_rcv_time_).seconds();
  size_t spoof_pub_count = raw_node_->count_publishers("/v2x/common/sensor_spoof/spoof");

  // アイコン更新 (通信状態)
  if (spoof_pub_count == 0) {
    // 誰もPublishしていない (自分も含む)
    spoof_icon_->updateStyle(IconState::Danger, QColor("#F08B8B"));
  } else if (spoof_elapsed > 3.0) {
    // データが流れていない
    spoof_icon_->updateStyle(IconState::Pending, QColor("#EEF08B"));
  } else {
    // 正常にデータ受信中
    spoof_icon_->updateStyle(IconState::Active, QColor("#8DF08B"));
  }

  // テキストとボタン更新 (True/Falseの値)
  if (spoof_elapsed > 3.0 && last_spoof_rcv_time_.nanoseconds() == 0) {
     // まだデータがない場合
     spoof_status_label_->setText("Spoof: UNKNOWN");
     spoof_status_label_->setStyleSheet("color: gray; font-weight: bold;");
     spoof_button_->setText("Turn ON");
  } else {
     // データがある場合
     if (latest_spoof_value_) {
       // TRUEの場合 (警告色)
       spoof_status_label_->setText("Spoof: TRUE (ON)");
       spoof_status_label_->setStyleSheet("color: #FF6666; font-weight: bold;"); // 赤っぽい色
       spoof_button_->setText("Turn OFF"); // ボタンは「切る」を表示
     } else {
       // FALSEの場合 (正常色)
       spoof_status_label_->setText("Spoof: FALSE (OFF)");
       spoof_status_label_->setStyleSheet("color: #8DFF8D; font-weight: bold;"); // 緑っぽい色
       spoof_button_->setText("Turn ON"); // ボタンは「入れる」を表示
     }
  }

  // ==========================================
  // 3. Auto Switch制御パネル更新ロジック
  // ==========================================
  double auto_switch_elapsed = (now - last_auto_switch_rcv_time_).seconds();
  size_t auto_switch_pub_count = raw_node_->count_publishers("/v2x/common/sensor_spoof/auto_switch");

  // アイコン更新 (通信状態)
  if (auto_switch_pub_count == 0) {
    // 誰もPublishしていない (自分も含む)
    auto_switch_icon_->updateStyle(IconState::Danger, QColor("#F08B8B"));
  } else if (auto_switch_elapsed > 3.0) {
    // データが流れていない
    auto_switch_icon_->updateStyle(IconState::Pending, QColor("#EEF08B"));
  } else {
    // 正常にデータ受信中
    auto_switch_icon_->updateStyle(IconState::Active, QColor("#8DF08B"));
  }

  // テキストとボタン更新 (True/Falseの値)
  if (auto_switch_elapsed > 3.0 && last_auto_switch_rcv_time_.nanoseconds() == 0) {
     // まだデータがない場合
     auto_switch_status_label_->setText("Auto Spoof Switch: UNKNOWN");
     auto_switch_status_label_->setStyleSheet("color: gray; font-weight: bold;");
     auto_switch_button_->setText("Turn ON");
  } else {
     // データがある場合
     if (latest_auto_switch_value_) {
       // TRUEの場合 (警告色)
       auto_switch_status_label_->setText("Auto Spoof Switch: TRUE (ON)");
       auto_switch_status_label_->setStyleSheet("color: #FF6666; font-weight: bold;"); // 赤っぽい色
       auto_switch_button_->setText("Turn OFF"); // ボタンは「切る」を表示
     } else {
       // FALSEの場合 (正常色)
       auto_switch_status_label_->setText("Auto Spoof Switch: FALSE (OFF)");
       auto_switch_status_label_->setStyleSheet("color: #8DFF8D; font-weight: bold;"); // 緑っぽい色
       auto_switch_button_->setText("Turn ON"); // ボタンは「入れる」を表示
     }
  }

  // ==========================================
  // 2. Override制御パネル更新ロジック
  // ==========================================
  double override_elapsed = (now - last_override_rcv_time_).seconds();
  size_t override_pub_count = raw_node_->count_publishers("/v2x/common/planning_override/enable");

  // アイコン更新 (通信状態)
  if (override_pub_count == 0) {
    // 誰もPublishしていない (自分も含む)
    override_icon_->updateStyle(IconState::Danger, QColor("#F08B8B"));
  } else if (override_elapsed > 3.0) {
    // データが流れていない
    override_icon_->updateStyle(IconState::Pending, QColor("#EEF08B"));
  } else {
    // 正常にデータ受信中
    override_icon_->updateStyle(IconState::Active, QColor("#8DF08B"));
  }

  // テキストとボタン更新 (True/Falseの値)
  if (override_elapsed > 3.0 && last_override_rcv_time_.nanoseconds() == 0) {
     // まだデータがない場合
     override_status_label_->setText("Override: UNKNOWN");
     override_status_label_->setStyleSheet("color: gray; font-weight: bold;");
     override_button_->setText("Turn ON");
  } else {
     // データがある場合
     if (latest_override_value_) {
       // TRUEの場合 (警告色)
       override_status_label_->setText("Override: TRUE (ON)");
       override_status_label_->setStyleSheet("color: #FF6666; font-weight: bold;"); // 赤っぽい色
       override_button_->setText("Turn OFF"); // ボタンは「切る」を表示
     } else {
       // FALSEの場合 (正常色)
       override_status_label_->setText("Override: FALSE (OFF)");
       override_status_label_->setStyleSheet("color: #8DFF8D; font-weight: bold;"); // 緑っぽい色
       override_button_->setText("Turn ON"); // ボタンは「入れる」を表示
     }
  }

  // ==========================================
  // 3. Delay UI更新
  // ==========================================
  double delay_elapsed = (now - last_delay_rcv_time_).seconds();
  size_t delay_pub_count = raw_node_->count_publishers("/v2x/common/delay");

  // 1. アイコン更新 (生存確認)
  if (delay_pub_count == 0) {
    delay_icon_->updateStyle(IconState::Danger, QColor("#F08B8B")); // 赤
  } else if (delay_elapsed > 3.0) {
    delay_icon_->updateStyle(IconState::Pending, QColor("#EEF08B")); // 黄
  } else {
    delay_icon_->updateStyle(IconState::Active, QColor("#8DF08B")); // 緑
  }

  // 2. 値の表示更新
  if (delay_elapsed > 3.0 && last_delay_rcv_time_.nanoseconds() == 0) {
    // データ未受信
    delay_label_->setText("Delay: WAITING");
    delay_label_->setStyleSheet("color: gray; font-weight: bold;");
  } else {
    // データ受信中
    // 小数点第3位まで表示する例
    QString val_str = QString::number(latest_delay_value_, 'f', 3);
    delay_label_->setText("Delay: " + val_str + "ms");
    
    // 値の大きさによって色を変えるなどのロジックも追加可能
    delay_label_->setStyleSheet("color: white; font-weight: bold;");
  }
}

} // namespace v2x_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(v2x_rviz_plugins::TopicMonitorPanel, rviz_common::Panel)