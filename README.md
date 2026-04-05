# Roadside Guardian

Roadside Guardian is a ROS 2 workspace designed to facilitate Vehicle-to-Everything (V2X) communication, specifically focusing on Road-Side Unit (RSU) and Autonomous Vehicle (Ego) interactions using Autoware. This project aims to demonstrate and experiment with V2X-based cooperative perception, trajectory arbitration, and sensor spoofing scenarios.

It acts as a top-level integration workspace containing essential packages and dependencies for both the RSU side and the Vehicle side.

## System Overview

The system bridges an Autonomous Vehicle (running Autoware) and an RSU (running perception sensors like LiDAR and Camera). The RSU can detect objects in blind spots, compute localized coordinates, and share perception data or corrected trajectories with the vehicle. Additionally, the workspace includes experimental "sensor spoofing" mechanisms to test the vehicle's response to unauthorized or faulty sensor injections.

### Component Architecture

The workspace is organized into two main domains: `rsu` and `vehicle`.

#### RSU (Infrastructure Side)
- **`rsu_autoware_bridge`**: Contains bridge nodes that handle the communication of trajectories and perception data between the RSU and the Vehicle. Evaluates and arbitrates vehicle trajectories based on V2X cooperative data.
- **`rsu_autoware_launcher`**: Custom launch configurations (`autoware_launch_rsu`) running the Autoware stack specifically structured for the RSU's planning or display modes.
- **`bbox2marker`**: An RViz2 helper utility that converts 3D bounding box detection data (`vision_msgs/Detection3DArray`) into visual markers (`visualization_msgs/MarkerArray`) for easy debugging.

#### Vehicle (Ego Side)
- **Vehicle Custom Launchers (`autoware_launch_v2x`, etc.)**: Configurations extending the standard `autoware_launch` tailored for the V2X interactions on the ego vehicle.
- **`sensor_publish_selector`**: A multiplexer node for sensor topics. It switches between the vehicle's `original` sensor data (e.g., LiDAR point cloud, camera images) and `fake` sensor data depending on the spoofing trigger flag.
- **`sensor_spoofing_trigger`**: Monitors the vehicle's kinematic state (odometry) and dynamically triggers a "spoofing" flag when the vehicle enters a specific geographical zone (e.g., past a defined Y-coordinate threshold).
- **`v2x_latency_monitor`**: Measures and publishes the round-trip or one-way communication latency between the vehicle and the RSU.

### External Dependencies
This workspace relies on several external repositories managed via `vcs` (vcstool), defined in `v2x_dependencies.repos`. Important dependencies include:
- **`rsu_preprocessor`**: Processes RSU LiDAR point clouds, performs ICP localization against a map, and broadcasts standard Autoware perception messages.
- **`fake_sensor_publisher`**: Plays back pre-recorded "fake" sensor data matching the vehicle's current pose to spoof the perception stack.
- **`v2x_dashboard`**: An OpenCV-based dashboard node to visualize ego vehicle status, V2X latency, spoofing activity, and planned trajectories on a top-down minimap.
- **`mmdet3d_ros2` & `mmdetection3d`**: Deep learning-based 3D object detection for the RSU's LiDAR.
- **`velodyne`**: Driver packages for the RSU's Velodyne LiDAR.
- **`autoware_auto_msgs`**: Standard message definitions used by the Autoware stack.

## Getting Started

### Prerequisites
- Ubuntu 22.04
- ROS 2 Humble
- Autoware Universe

### Installation

1. Clone this repository (which acts as the workspace root):
```bash
git clone https://github.com/sakunaka39/roadside-guardian.git
cd roadside-guardian
```

2. Import dependencies using `vcs`:
```bash
vcs import src < v2x_dependencies.repos
```

3. Install ROS dependencies:
```bash
rosdep update
rosdep install -i --from-path src --rosdistro humble -y
```

4. Source your installed Autoware workspace and build the repository:
> **Important**: You MUST source your Autoware workspace (e.g., `~/autoware/install/setup.bash`) before building this repository, as it depends on Autoware messages and libraries.

```bash
source ~/autoware/install/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Usage

The workspace provides interactive start scripts to easily launch different configurations for both the RSU and the Vehicle.

### 1. Environment Configuration (`setup-ros-env-rsu.sh`)

Before launching any nodes, you must configure your ROS 2 domain to prevent crosstalk and appropriately isolate the RSU and Ego vehicle networks. **Note**: You must source this script in *every* new terminal you open before running the subsequent commands.

```bash
source setup-ros-env-rsu.sh
```

You will be prompted to select a mode (Vehicle, RSU, or Standalone). This correctly sets your `ROS_DOMAIN_ID` and `ROS_LOCALHOST_ONLY` environment variables.

### 2. Execution Workflow

For both the Vehicle (PIXKIT) and the RSU, you will need to open **three separate terminals** and run the following scripts in sequence (1 -> 2 -> 3).

#### Terminal 1: Establish Zenoh V2X Bridge (`connect_zenoh.sh`)

Use the `connect_zenoh.sh` script to establish the Zenoh-based bridge for cross-domain communication between the Vehicle and the RSU:

```bash
./connect_zenoh.sh
```

You will be prompted to select a mode to connect as either the Ego Vehicle or the RSU.

#### Terminal 2: Launch Autoware (`launch_autoware.sh`)

Use the `launch_autoware.sh` script to launch the core Autoware stack in different modes:

```bash
./launch_autoware.sh
```

You will be prompted to select a mode:
1. **PIXKIT Mode**: Launches Autoware for the ego vehicle (PIXKIT).
2. **RSU Planning Mode**: Launches the Autoware planning stack on the RSU to calculate alternative trajectories based on cooperative data.

#### Terminal 3: Launch V2X Nodes (`launch_v2x.sh`)

Use the `launch_v2x.sh` script to launch the V2X, sensing, and bridge nodes that connect the RSU and the vehicle:

```bash
./launch_v2x.sh
```

You will be prompted to select a mode:
1. **PIXKIT Mode**: Launches vehicle-side nodes, including the V2X bridge, sensor publisher selectors, spoofing trigger, latency monitor, and dashboard.
2. **RSU Mode**: Launches RSU-side nodes. This includes executing helper scripts (`scripts/launch_vlp32.sh`, `scripts/launch_lidar_detection.sh` for multi-sensor detection), the RSU preprocessor (`rsu_preprocessor.launch.xml`), and the RSU-side V2X bridge.

### 3. Monitoring
When running the Vehicle V2X mode, the `v2x_dashboard` will automatically launch to visualize the V2X communication latency, trajectory overrides, and sensor spoofing states in real-time.

## Important Notes

- **RSU Sensor Configuration**: The RSU perception stack and scripts are specifically designed assuming the use of **Velodyne LiDAR** (e.g., VLP-16 or VLP-32C). Using a different sensor will require adjusting the driver logic and parameters.
- **Message Type Discrepancy**: This experimental setup is intentionally designed to bridge two different message ecosystems. The vehicle side (PIXKIT) operates using the older `autoware_auto_msgs`, whereas the RSU side uses the latest Autoware framework utilizing `autoware_msgs`.
- **Map Data Not Included**: Map data (Point cloud maps and Lanelet2 maps) is **not included** in this repository. You must prepare your own maps and update the map paths accordingly.
- **Script Path Customization**: The launch scripts (`launch_autoware.sh`, `launch_v2x.sh`, etc.) contain specific workspace and map paths (e.g., `MAP_PATH` or `source ~/autoware/install/setup.bash`). Please edit these `.sh` files and adjust the paths to match your local environment before executing them.

---
*Disclaimer: This repository is intended for research, testing, and simulation purposes regarding V2X communication and cybersecurity.*

## Acknowledgments and Licenses

The custom launch and configuration packages within this repository are heavily dependent on and based upon the open-source **[Autoware](https://github.com/autowarefoundation/autoware)** project. 

By design, this repository does not duplicate the entire Autoware system. Instead, it employs a sparse, overlay-based architecture that contains only the specific launch files and parameter configurations modified for our Road-Side Unit (RSU) and V2X vehicle environments. This ensures seamless integration and tracking with upstream Autoware updates.

In accordance with the original source material, all such modified packages and overlay configurations inherit and are distributed under the **Apache License 2.0**. Please refer to the respective package's `package.xml` and source files for specific copyright headers.

We would like to express our profound gratitude to the **Autoware Foundation** and **Tier IV, Inc.** for providing the robust, state-of-the-art baseline architecture that made this research and development possible.