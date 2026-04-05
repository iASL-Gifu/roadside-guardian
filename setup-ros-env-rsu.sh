#!/bin/bash

# ==========================================
# ROS 2 Environment Setup Script
# ==========================================

# Note: This script must be "sourced" to apply environment variables
# to the current shell.
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "Error: This script must be run using the 'source' command."
    echo "Usage: source setup-ros-env.sh"
    exit 1
fi

# --- Configuration Values (Modify as needed) ---
ID_VEHICLE=40     # Domain ID for Vehicle
ID_RSU=39         # Domain ID for RSU
ID_STANDALONE=39  # Domain ID for Standalone

if [ -n "$1" ]; then
    choice="$1"
    echo "Running with argument: $choice"
else
    echo "----------------------------------------"
    echo "Configuring ROS 2 Communication Environment."
    echo "----------------------------------------"
    echo "1) Vehicle Mode"
    echo "   -> Domain ID: $ID_VEHICLE, Localhost Only: 0 (False)"
    echo ""
    echo "2) RSU Mode"
    echo "   -> Domain ID: $ID_RSU, Localhost Only: 0 (False)"
    echo ""
    echo "3) Standalone/Test Mode"
    echo "   -> Domain ID: $ID_STANDALONE,  Localhost Only: 1 (True)"
    echo "----------------------------------------"
    echo -n "Enter the number of your choice [1-3]: "
    read choice
fi

case $choice in
    1)
        export ROS_DOMAIN_ID=$ID_VEHICLE
        export ROS_LOCALHOST_ONLY=0
        MODE_NAME="Vehicle"
        ;;
    2)
        export ROS_DOMAIN_ID=$ID_RSU
        export ROS_LOCALHOST_ONLY=0
        MODE_NAME="RSU"
        ;;
    3)
        export ROS_DOMAIN_ID=$ID_STANDALONE
        export ROS_LOCALHOST_ONLY=1
        MODE_NAME="Standalone"
        ;;
    *)
        echo "Invalid selection. Configuration aborted."
        return 1 2>/dev/null || exit 1
        ;;
esac

echo ""
echo "✅ Setup Complete: $MODE_NAME"
echo "   export ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "   export ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
echo "----------------------------------------"

# For verification (optional)
# printenv | grep ROS_
