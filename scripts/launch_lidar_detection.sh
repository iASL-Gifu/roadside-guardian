#!/bin/bash

source ~/roadside-guardian/install/setup.bash

# 3センサの物体検出ノードを並列起動
ros2 launch mmdet3d_ros2 mmdet3d_infer_launch.py model_name:="ssn" &
ros2 run bbox2marker detection3d_to_marker_node &

# 少し待機（ノードが起動してからRVizを起動）
sleep 5

# RViz起動（設定ファイルを読み込み）
rviz2 -d ~/roadside-guardian/scripts/display_3detection.rviz &

wait

