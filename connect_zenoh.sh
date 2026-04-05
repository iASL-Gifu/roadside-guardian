#!/bin/bash

# ==========================================
# ROS 2 Zenoh Connection Script
# ==========================================

# --- Configuration ---
# (Note: Variables defined here for reference, currently direct paths are used below)
VEHICLE_CONFIG="./zenoh/vehicle.yaml"
RSU_CONFIG="./zenoh/rsu.yaml"

echo "----------------------------------------"
echo "Configuring Zenoh Communication Environment."
echo "----------------------------------------"
echo "1) Vehicle Mode"
echo ""
echo "2) RSU Mode"
echo "----------------------------------------"
echo -n "Enter the number of your choice [1-2]: "
read choice

case $choice in
    1)
        MODE_NAME="Vehicle"
        echo "Starting Zenoh Bridge for $MODE_NAME..."
        zenoh-bridge-ros2dds -c $VEHICLE_CONFIG
        ;;
    2)
        MODE_NAME="RSU"
        echo "Starting Zenoh Bridge for $MODE_NAME..."
        zenoh-bridge-ros2dds -c $RSU_CONFIG
        ;;
    *)
        echo "Invalid selection. Aborting."
        return 1 2>/dev/null || exit 1
        ;;
esac
