# Phase 0 - Project Analysis Report
**Author:** Uma  
**Date:** Phase 0 Analysis  
**Environment:** ROS 2 Jazzy + Gazebo Harmonic + Docker Container

---

## 1. ROS Topics Analysis

### 1.1 Core Robot Topics

#### **Input Topics (Robot Subscribes To):**
- **`/cmd_vel`** (geometry_msgs/msg/Twist)
  - **Publisher:** `saye_control::Joystick_Controller`, `ackermann_drl::AckermannCityEnv`
  - **Subscriber:** Gazebo AckermannSteering plugin (via bridge)
  - **Direction:** ROS → Gazebo (BIDIRECTIONAL in bridge)
  - **Usage:** Linear velocity (x) and angular velocity (z) for Ackermann steering

#### **Output Topics (Robot Publishes):**
- **`/odom`** (nav_msgs/msg/Odometry)
  - **Publisher:** Gazebo OdometryPublisher plugin (via bridge)
  - **Subscribers:** `saye_localization::kalman_filter`, `ackermann_drl::AckermannCityEnv`, `saye_bringup::odom_to_tf`
  - **Direction:** Gazebo → ROS (GZ_TO_ROS)
  - **Frame:** `odom` → `saye/base_link`
  - **Usage:** Robot pose and velocity estimation

- **`/scan`** (sensor_msgs/msg/LaserScan)
  - **Publisher:** Gazebo LiDAR sensor (via bridge)
  - **Subscribers:** `ackermann_drl::AckermannCityEnv`, SLAM toolbox, Nav2
  - **Direction:** Gazebo → ROS (GZ_TO_ROS)
  - **Usage:** 720-point laser scan for obstacle detection and mapping

- **`/imu`** (sensor_msgs/msg/Imu)
  - **Publisher:** Gazebo IMU sensor (via bridge)
  - **Subscribers:** `saye_localization::kalman_filter`
  - **Direction:** Gazebo → ROS (GZ_TO_ROS)
  - **Usage:** Angular velocity (z-axis) for Kalman filter fusion

- **`/tf`** (tf2_msgs/msg/TFMessage)
  - **Publisher:** Gazebo PosePublisher plugin (via bridge)
  - **Direction:** Gazebo → ROS (GZ_TO_ROS)
  - **Usage:** Transform tree for robot pose

- **`/camera/front_raw`** (sensor_msgs/msg/Image)
  - **Publisher:** Gazebo camera sensor (via bridge)
  - **Direction:** Gazebo → ROS (GZ_TO_ROS)
  - **Usage:** Front camera feed (optional for DRL)

#### **Additional Topics:**
- **`/joy`** (sensor_msgs/msg/Joy)
  - **Publisher:** External joystick node
  - **Subscriber:** `saye_control::Joystick_Controller`
  - **Usage:** Manual joystick control

- **`/laser_scan`** (saye_msgs/msg/Map)
  - **Publisher:** (Placeholder - not currently used)
  - **Subscriber:** `saye_control::Control`
  - **Usage:** Custom map message (future use)

- **`/map`** (nav_msgs/msg/OccupancyGrid)
  - **Publisher:** SLAM toolbox, `saye_control::Control`
  - **Subscribers:** Nav2, RViz, map_saver
  - **Usage:** 2D occupancy grid for navigation

- **`/robot/odom_kalman`** (nav_msgs/msg/Odometry)
  - **Publisher:** `saye_localization::kalman_filter`
  - **Usage:** Filtered odometry with Kalman-filtered angular velocity

- **`publish_map_service`** (saye_msgs/srv/ShareMap)
  - **Server:** `saye_control::Control`
  - **Client:** `saye_control::client.cpp`
  - **Usage:** Map sharing service (placeholder)

---

## 2. TF Frames Analysis

### 2.1 Frame Hierarchy

```
map (from SLAM/Nav2)
  └── odom (from Gazebo OdometryPublisher plugin)
      └── saye (from odom_to_tf.py)
          └── saye/base_link (from robot_state_publisher)
              ├── saye/lidar_link (from robot description)
              ├── saye/imu_sensor (from robot description)
              └── saye/camera_link (from robot description)
```

### 2.2 Frame Details

- **`map`**
  - **Publisher:** SLAM toolbox (when mapping) or Nav2 (when localizing)
  - **Usage:** Global map frame for navigation

- **`odom`**
  - **Publisher:** Gazebo OdometryPublisher plugin (publishes TF directly)
  - **Child:** `saye/base_link` (via plugin configuration)
  - **Usage:** Odometry frame (drift over time)

- **`saye`**
  - **Publisher:** `saye_bringup::odom_to_tf.py` (extracts from `/odom` message)
  - **Purpose:** Intermediate frame to connect `odom` → `saye/base_link`
  - **Note:** robot_state_publisher uses `frame_prefix: 'saye/'`

- **`saye/base_link`**
  - **Publisher:** robot_state_publisher (from URDF/SDF)
  - **Parent:** `saye` (via robot_state_publisher)
  - **Usage:** Base frame of the robot

- **`saye/lidar_link`**
  - **Publisher:** robot_state_publisher
  - **Parent:** `saye/base_link`
  - **Usage:** LiDAR sensor frame

- **`saye/imu_sensor`**
  - **Publisher:** robot_state_publisher
  - **Parent:** `saye/base_link`
  - **Usage:** IMU sensor frame

### 2.3 TF Configuration

- **robot_state_publisher:**
  - **Frame prefix:** `saye/`
  - **Input:** URDF from `saye_description/urdf/saye.urdf`
  - **Output:** Publishes `saye/` → `saye/base_link` and child frames

- **odom_to_tf.py:**
  - **Input:** `/odom` topic
  - **Output:** `odom` → `saye` transform
  - **Purpose:** Ensures TF chain connectivity when Gazebo plugin doesn't publish TF

---

## 3. Docker Container Analysis

### 3.1 Docker Setup

**Dockerfile Location:** `/home/studente/ackermann_sim/src/ackermann-vehicle-gzsim-ros2/Dockerfile`

**Base Image:** `ros:jazzy`

**Key Environment Variables:**
- `COLCON_WS=/root/colcon_ws`
- `COLCON_WS_SRC=/root/colcon_ws/src`
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- `ROS_DISABLE_SHARED_MEMORY=1`
- `GZ_SIM_RESOURCE_PATH` (includes project models)
- `ROS_PACKAGE_PATH` (includes project packages)

**Docker Compose:**
- **Service:** `ackermann_sim`
- **Container Name:** `ackermann_sim`
- **Working Directory:** `/root/colcon_ws`
- **Volumes:**
  - Host source → `/root/colcon_ws/src/ackermann-vehicle-gzsim-ros2`
  - X11 socket for GUI
  - User runtime

**Graphics Configuration:**
- Software rendering (MESA/llvmpipe)
- Vulkan support for Gazebo Harmonic
- X11 forwarding for GUI

### 3.2 Gazebo Launch Process in Docker

**Launch Command:**
```bash
ros2 launch saye_bringup saye_spawn.launch.py gui:=true
```

**Launch File:** `saye_bringup/launch/saye_spawn.launch.py`

**Process Flow:**
1. **Gazebo Sim Launch:**
   - Uses `ros_gz_sim::gz_sim.launch.py`
   - World file: `saye_description/worlds/bari_world.sdf` (default)
   - GUI: Controlled by `gui` parameter

2. **ROS-Gazebo Bridge:**
   - Node: `ros_gz_bridge::parameter_bridge`
   - Config: `saye_bringup/config/ros_gz_bridge.yaml`
   - Bridges topics: `/cmd_vel`, `/odom`, `/scan`, `/imu`, `/tf`, `/camera/front_raw`

3. **Robot State Publisher:**
   - Publishes TF from URDF
   - Frame prefix: `saye/`

4. **Robot Spawn:**
   - Node: `ros_gz_sim::create`
   - SDF file: `saye_description/models/saye/model.sdf`
   - Default pose: `x=169.37, y=0.21, z=0.35, yaw=0.0796` (Via Dante, spawn_point_173)

5. **Camera Setup:**
   - Delayed action (4s after spawn)
   - Uses `gz service` to set camera pose
   - Position: `164.37, 0.21, 3.35` (5m behind, 3m above robot)

### 3.3 Spawn Process Details

**Spawn Parameters (from launch file):**
- `robot_x`: 169.37 (east coordinate, meters)
- `robot_y`: 0.21 (north coordinate, meters)
- `robot_z`: 0.35 (height, meters)
- `robot_R`: 0.0 (roll, radians)
- `robot_P`: 0.0 (pitch, radians)
- `robot_Y`: 0.0796 (yaw, radians)

**Spawn Location:**
- Street: Via Dante Alighieri (tertiary road near city center)
- Spawn Point ID: 173 (from `osm_city_pipeline/maps/bari_spawn_points.yaml`)
- Coordinates: ENU (East-North-Up) format from OSM projection

**World File:**
- Location: `saye_description/worlds/bari_world.sdf`
- Includes: `bari_3d` model (3D city mesh from OSM2World)
- Camera: Pre-configured at spawn location

---

## 4. Street Coordinates Storage

### 4.1 OSM City Pipeline Outputs

**Location:** `/home/studente/ackermann_sim/src/osm_city_pipeline/maps/`

**Files:**
1. **`bari_roads.json`**
   - **Format:** JSON
   - **Content:**
     - `projection_center`: {latitude, longitude, height}
     - `roads`: Array of road objects
       - `way_id`: OSM way ID
       - `name`: Street name
       - `highway_type`: Road type (motorway, primary, secondary, etc.)
       - `lanes`: Number of lanes
       - `centerline_enu`: Array of {east, north, up} coordinates
   - **Usage:** Road geometry for navigation and path planning
   - **Accessed by:** `ackermann_drl/utils/roads_geometry.py`

2. **`bari_spawn_points.yaml`**
   - **Format:** YAML
   - **Content:**
     - `spawn_points`: Array of spawn point objects
       - `id`: Unique spawn point ID
       - `name`: Spawn point name
       - `position`: {east, north, up}
       - `orientation`: {yaw}
       - `way_id`: OSM way ID
       - `road_name`: Street name
       - `highway_type`: Road type
     - `total_spawn_points`: Count
     - `projection_center`: {latitude, longitude, height}
   - **Usage:** Robot spawn positions on roads
   - **Accessed by:** `saye_bringup/launch/saye_spawn.launch.py` (default spawn)

### 4.2 Coordinate System

**Projection:** ENU (East-North-Up)
- **East (X):** Positive = eastward
- **North (Y):** Positive = northward
- **Up (Z):** Positive = upward

**Projection Center:**
- Extracted from OSM file bounding box
- Used by `osm_city_pipeline/src/osm_city_pipeline/gis_projection.py`
- Stored in both JSON and YAML files

**Integration:**
- `ackermann_drl/utils/roads_geometry.py` loads `bari_roads.json`
- Provides functions: `get_road_by_name()`, `get_road_centerline()`
- Used for DRL environment road-aware navigation

---

## 5. DRL Integration Points

### 5.1 Current DRL Package Structure

**Package:** `ackermann_drl`

**Location:** `/home/studente/ackermann_sim/src/ackermann-vehicle-gzsim-ros2/ackermann_drl/`

**Components:**
1. **Environment:** `envs/ackermann_city_env.py`
   - ROS 2 node wrapping simulation
   - Subscribes: `/scan`, `/odom`
   - Publishes: `/cmd_vel`
   - Methods: `reset()`, `step()`, `get_observation()`

2. **Training Script:** `scripts/train_ppo.py` (placeholder)

3. **Evaluation Script:** `scripts/eval_policy.py` (placeholder)

4. **Utilities:**
   - `utils/battery_model.py`: Energy consumption tracking
   - `utils/reset_helpers.py`: Robot pose reset
   - `utils/roads_geometry.py`: Road data loading

5. **Launch:** `launch/single_drl_training.launch.py`

6. **Config:** `config/drl_params.yaml`

### 5.2 Integration Requirements

**MUST NOT BREAK:**
- Existing ROS topics (`/cmd_vel`, `/odom`, `/scan`, `/imu`)
- TF frame hierarchy
- Gazebo simulation
- SLAM/Nav2 functionality
- Joystick control

**DRL Integration Points:**
1. **Environment Node:**
   - Already exists: `AckermannCityEnv`
   - Must be enhanced with:
     - Gymnasium wrapper
     - Reward function
     - Episode termination conditions
     - Action/observation space definition

2. **Training Script:**
   - Must launch Gazebo + robot + environment
   - Must support single robot only
   - Must run inside Docker container

3. **Reset Mechanism:**
   - Must reset robot pose to spawn points
   - Must reset simulation state
   - Must use `bari_spawn_points.yaml` for spawn selection

4. **Observation Space:**
   - Laser scan: 720 points (already implemented)
   - Odometry: Position, velocity, orientation
   - Optional: Camera images, road geometry

5. **Action Space:**
   - Linear velocity: [-max_vel, max_vel]
   - Angular velocity: [-max_ang_vel, max_ang_vel]
   - Or: Steering angle + throttle

### 5.3 Safe Integration Locations

**Recommended Integration Points:**
1. **New Launch File:** `ackermann_drl/launch/drl_training.launch.py`
   - Launches: Gazebo + robot + DRL environment
   - Does NOT launch: SLAM, Nav2, joystick (unless needed)

2. **Enhanced Environment:** `ackermann_drl/envs/ackermann_city_env.py`
   - Add Gymnasium wrapper
   - Add reward/termination logic
   - Keep ROS 2 node structure

3. **Training Script:** `ackermann_drl/scripts/train_ppo.py`
   - Uses stable-baselines3 or similar
   - Runs training loop
   - Handles reset/step cycle

4. **Config File:** `ackermann_drl/config/drl_params.yaml`
   - Training hyperparameters
   - Environment parameters
   - Spawn point selection

**Files to AVOID Modifying:**
- `saye_bringup/launch/saye_spawn.launch.py` (keep for manual testing)
- `saye_control/` (keep for joystick control)
- `saye_localization/` (keep for Kalman filter)
- `saye_description/models/saye/model.sdf` (robot model)
- `saye_description/worlds/bari_world.sdf` (world file)

---

## 6. Dependencies Analysis

### 6.1 Python Dependencies (for DRL)

**Core DRL Libraries:**
- `gymnasium>=0.29.0` (Gym API for RL environments)
- `stable-baselines3>=2.0.0` (PPO and other algorithms)
- `torch>=2.0.0` (PyTorch for neural networks)
- `numpy>=1.24.0` (Numerical operations - already installed via apt)

**ROS 2 Integration:**
- `rclpy` (already installed via ROS 2 Jazzy)
- `sensor_msgs` (already installed)
- `nav_msgs` (already installed)
- `geometry_msgs` (already installed)

**Utilities:**
- `pyyaml>=6.0` (YAML parsing - already installed via apt)
- `matplotlib>=3.7.0` (Optional: for visualization)
- `tensorboard>=2.13.0` (Optional: for training logs)

**Note:** `numpy` is already installed via `python3-numpy` in Dockerfile.

### 6.2 APT Dependencies

**Already Installed (from Dockerfile):**
- `python3-pip` (Python package manager)
- `python3-numpy` (NumPy)
- `python3-yaml` (PyYAML)
- `build-essential` (C++ compiler for torch)
- `mesa-utils`, `libglvnd0`, `vulkan-tools` (Graphics support)

**Additional APT Dependencies Needed:**
- **NONE** - All required packages are already installed or will be installed via pip.

**Note:** PyTorch will be installed via pip (CPU or CUDA version as needed).

### 6.3 Dockerfile Updates Required

**Current Dockerfile:** `/home/studente/ackermann_sim/src/ackermann-vehicle-gzsim-ros2/Dockerfile`

**Required Updates:**

1. **Add Python DRL Dependencies (after line 20, before colcon build):**
```dockerfile
# Install Python DRL dependencies
RUN pip3 install --no-cache-dir --break-system-packages \
    gymnasium>=0.29.0 \
    stable-baselines3>=2.0.0 \
    torch>=2.0.0 \
    matplotlib>=3.7.0 \
    tensorboard>=2.13.0
```

2. **No APT changes needed** - all system dependencies are already present.

3. **Environment Variables (optional, for PyTorch):**
```dockerfile
# PyTorch environment (CPU version by default)
ENV TORCH_CUDA_ARCH_LIST=""
```

**Note:** Use `--break-system-packages` flag as we're in a container environment (similar to osm_city_pipeline Dockerfile).

---

## 7. Docker Container Execution

### 7.1 DRL Must Run Inside Container

**CONFIRMED:** DRL training MUST run inside the Docker container.

**Reasons:**
1. All ROS 2 topics are only available inside the container
2. Gazebo simulation runs inside the container
3. ROS-Gazebo bridge is configured inside the container
4. All dependencies (ROS 2, Gazebo, Python packages) are installed in the container
5. Volume mount ensures code changes are reflected immediately

**Execution Flow:**
1. Start container: `docker compose up -d`
2. Enter container: `docker compose exec ackermann_sim bash`
3. Source workspace: Already sourced in `.bashrc`
4. Run training: `ros2 launch ackermann_drl drl_training.launch.py`

### 7.2 Container Constraints

**MUST PRESERVE:**
- ROS 2 Jazzy installation
- Gazebo Harmonic installation
- rviz2 functionality
- Existing simulation capabilities

**MUST NOT:**
- Install packages on host WSL
- Break existing ROS topics
- Modify core robot/world files unnecessarily
- Install conflicting Python packages

---

## 8. Summary

### 8.1 Key Findings

1. **ROS Topics:**
   - Input: `/cmd_vel` (Twist)
   - Output: `/odom`, `/scan`, `/imu`, `/tf`, `/camera/front_raw`
   - All bridged via `ros_gz_bridge`

2. **TF Frames:**
   - Hierarchy: `map` → `odom` → `saye` → `saye/base_link`
   - robot_state_publisher uses `frame_prefix: 'saye/'`

3. **Docker:**
   - Container: `ackermann_sim`
   - Working dir: `/root/colcon_ws`
   - Gazebo launched via `ros2 launch saye_bringup saye_spawn.launch.py`

4. **Street Coordinates:**
   - `osm_city_pipeline/maps/bari_roads.json` (road centerlines)
   - `osm_city_pipeline/maps/bari_spawn_points.yaml` (spawn points)
   - ENU coordinate system

5. **DRL Integration:**
   - Package: `ackermann_drl`
   - Environment: `AckermannCityEnv` (needs Gymnasium wrapper)
   - Must run inside Docker container
   - Must support single robot only

### 8.2 Dependencies Summary

**Python (pip install):**
- gymnasium>=0.29.0
- stable-baselines3>=2.0.0
- torch>=2.0.0
- matplotlib>=3.7.0
- tensorboard>=2.13.0

**APT:**
- None (all already installed)

**Dockerfile Updates:**
- Add pip install for DRL libraries (before colcon build)

### 8.3 Next Steps (Future Phases)

1. **Phase 1:** Update Dockerfile with DRL dependencies
2. **Phase 2:** Enhance `AckermannCityEnv` with Gymnasium wrapper
3. **Phase 3:** Implement reward function and termination conditions
4. **Phase 4:** Implement PPO training script
5. **Phase 5:** Add reset mechanism using spawn points
6. **Phase 6:** Testing and validation

---

## 9. Verification Checklist

- [x] ROS topics identified and documented
- [x] TF frames identified and documented
- [x] Docker setup analyzed
- [x] Gazebo launch process documented
- [x] Spawn process documented
- [x] Street coordinates location identified
- [x] DRL integration points identified
- [x] Python dependencies listed
- [x] APT dependencies listed (none needed)
- [x] Dockerfile updates identified
- [x] Container execution confirmed
- [x] No breaking changes identified

---

**END OF PHASE 0 ANALYSIS**

