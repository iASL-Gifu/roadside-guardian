#!/bin/bash

# ==========================================
# Launch Autoware Script
# ==========================================

# --- Configuration ---
# (Note: Variables defined here for reference, currently direct paths are used below)
MAP_PATH="./src/autoware_map/map01"

echo "----------------------------------------"
echo "Configuring Autoware Environment."
echo "----------------------------------------"
echo "1) PIXKIT Mode"
echo ""
echo "2) RSU Planning Mode"
echo "----------------------------------------"
echo -n "Enter the number of your choice [1-2]: "
read choice

case $choice in
    1)
        MODE_NAME="PIXKIT"
        echo "Launching Autoware for $MODE_NAME..."
        
        source ~/autoware/install/setup.bash
        source install/setup.bash
        ros2 launch autoware_launch_v2x autoware.launch.xml map_path:=$MAP_PATH vehicle_model:=pixkit sensor_model:=pixkit_sensor_kit
        ;;
    2)
        MODE_NAME="RSU Planning"
        echo "Launching Autoware for $MODE_NAME..."
        
        source ~/autoware/install/setup.bash
        source install/setup.bash
        ros2 launch autoware_launch_rsu autoware.launch.xml \
        map_path:=$MAP_PATH \
        vehicle_model:=sample_vehicle \
        sensor_model:=sample_sensor_kit \
        launch_vehicle:=true \
        launch_system:=false \
        launch_map:=true \
        launch_sensing:=false \
        launch_localization:=false \
        launch_perception:=false \
        launch_planning:=true \
        launch_control:=false \
        enable_all_modules_auto_mode:=true \
        rviz_config:=./src/rsu_autoware_launcher/autoware_launch_rsu/rsu_planning2.rviz
        ;;
    *)
        echo "Invalid selection. Aborting."
        return 1 2>/dev/null || exit 1
        ;;
esac
