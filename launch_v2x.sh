#!/bin/bash

# ==========================================
# Launch Autoware Script
# ==========================================

# --- Configuration ---
# (Note: Variables defined here for reference, currently direct paths are used below)

echo "----------------------------------------"
echo "Configuring Autoware Environment."
echo "----------------------------------------"
echo "1) PIXKIT Mode"
echo ""
echo "2) RSU Mode"
echo "----------------------------------------"
echo -n "Enter the number of your choice [1-2]: "
read choice

case $choice in
    1)
        MODE_NAME="PIXKIT"
        echo "Launching Autoware for $MODE_NAME..."
        
        source ~/autoware/install/setup.bash
        source install/setup.bash
        ros2 run rsu_autoware_bridge vehicle_bridge_node &
        ros2 run rsu_autoware_bridge vehicle_trajectory_arbiter_node &
        ros2 run fake_sensor_publisher fake_sensor_publisher &
        ros2 run sensor_publish_selector lidar_selector_node &
        ros2 run sensor_publish_selector camera_selector_node &
        ros2 run v2x_latency_monitor latency_monitor_node &
        ros2 run v2x_dashboard dashboard_node &
        ros2 run sensor_spoofing_trigger spoofing_trigger_node
        ;;
    2)
        MODE_NAME="RSU"
        echo "Launching Autoware for $MODE_NAME..."
        
        source install/setup.bash
        scripts/launch_vlp32.sh &
        scripts/launch_lidar_detection.sh &
        ros2 launch rsu_preprocessor rsu_preprocessor.launch.xml &
        ros2 run rsu_autoware_bridge rsu_bridge_node &
        ros2 run topic_tools relay /v2x/vehicle/tf /tf
        ;;
    *)
        echo "Invalid selection. Aborting."
        return 1 2>/dev/null || exit 1
        ;;
esac
