#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source ~/roadside-guardian/install/setup.bash
ros2 launch velodyne velodyne-all-nodes-VLP32C-launch.py
