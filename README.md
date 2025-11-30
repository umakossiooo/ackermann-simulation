# Ackermann Steering Vehicle Simulation (ROS 2 + Gazebo Sim Harmonic)

Minimal ROS 2/Gazebo Harmonic setup for an Ackermann car with IMU, LiDAR, cameras, SLAM, and Nav2 already wired together.

**Default map:** The Bari world (`saye_description/worlds/bari_world.sdf`) and its 2D Nav2 map (`saye_bringup/maps/bari_map.yaml`) load by default. The vehicle now spawns on Via Andrea da Bari (city center), using coordinates exported by `osm_city_pipeline`. Override `map:=...` or the `robot_*` launch arguments only if you need a different location.

## Requirements
- ROS 2 Jazzy (or Humble) with Nav2, RViz2, and `ros-gz`
- Gazebo Sim Harmonic
- Optional: Docker, Docker Compose

## Local Setup
- Install Gazebo ↔ ROS bridge packages (once):
  ```bash
  sudo apt-get install ros-${ROS_DISTRO}-ros-gz ros-${ROS_DISTRO}-ros-gzharmonic
  ```
- Create a workspace and clone the repo:
  ```bash
  mkdir -p ~/ackermann_sim/src && cd ~/ackermann_sim/src
  git clone https://github.com/umakossiooo/ackermann-simulation.git
  cd ..
  ```
- Build and source the overlay:
  ```bash
  colcon build && source install/setup.bash
  ```
- Make sure Gazebo can find the Bari models (run per shell or add to `.bashrc`):
  ```bash
  export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/ackermann_sim/src/ackermann-vehicle-gzsim-ros2/
  export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/ackermann_sim/src/ackermann-vehicle-gzsim-ros2/
  ```

## Run Locally (Bari by default)

### Spawn (Gazebo + RViz bridge)
Start Gazebo Harmonic in the Bari world with the default ROS 2 bridges.
```bash
ros2 launch saye_bringup saye_spawn.launch.py gui:=true
```

### Navigation (Nav2 + AMCL on Bari map)
Bring up Nav2 + AMCL against the saved Bari occupancy grid.
```bash
ros2 launch saye_bringup navigation_bringup.launch.py \
  map:=~/ackermann_sim/src/ackermann-vehicle-gzsim-ros2/saye_bringup/maps/bari_map.yaml
```

### SLAM (slam_toolbox mapping only)
Run slam_toolbox to build a new 2D map while driving manually or via Nav2.
```bash
ros2 launch saye_bringup slam.launch.py gui:=true
```

### SLAM + Nav2 (synchronous mapping + planning)
Launch Gazebo, slam_toolbox, Nav2 planner/controller, RViz, and map saver in one shot.
```bash
ros2 launch saye_bringup slam_navigation.launch.py gui:=true
```

### Teleop (optional driving input)
Send velocity commands from the keyboard teleop node.
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Save the Bari occupancy grid
**IMPORTANT:** You must run this command **WHILE SLAM is running** (e.g., while `slam.launch.py` or `slam_navigation.launch.py` is active).

**Prerequisites:**
1. **Verify SLAM is running and publishing `/map`:**
   ```bash
   ros2 topic list | grep /map
   ros2 topic hz /map  # Should show publication rate
   ```
   If `/map` doesn't appear, SLAM may not be running or the robot needs to move first.

2. **Dependencies:** The Docker image includes all required packages (`python3-numpy`, `python3-pil`, `python3-yaml`, `slam-toolbox`). If you're running locally, install them:
   ```bash
   sudo apt-get install -y python3-numpy python3-pil python3-yaml ros-${ROS_DISTRO}-slam-toolbox
   ```

**Method 1: Using the provided script (Recommended)**
```bash
# In Docker container, use the colcon_ws path (volume is mounted there):
python3 /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/saye_bringup/scripts/save_map.py \
  /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/saye_bringup/maps/bari_map
```

**Method 2: Using the service (if map_saver_server is running)**
First check if the service exists:
```bash
ros2 service list | grep map_saver
```

If you see `/map_saver/save_map`, then call it with the correct format:
```bash
ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap \
  "{map_url: '/root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/saye_bringup/maps/bari_map'}"
```

**Note:** The `map_url` field should be a string, not a nested structure. If the service returns `result: False`, check:
1. The `/map` topic is being published: `ros2 topic list | grep /map`
2. The map_saver node is active (check lifecycle state)
3. The directory exists and is writable

**Method 3: Using map_saver_cli (may require lifecycle activation)**
```bash
ros2 run nav2_map_server map_saver_cli \
  -f /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/saye_bringup/maps/bari_map
```

**Troubleshooting:**
- **If `/map` topic doesn't exist:**
  1. Make sure SLAM launch is running: `ros2 launch saye_bringup slam.launch.py gui:=true`
  2. Wait 10-20 seconds for SLAM to initialize
  3. Move the robot a bit (SLAM needs movement to start mapping)
  4. Check SLAM logs for errors: look for "slam_toolbox" in the terminal output
  5. Verify `/scan` topic exists: `ros2 topic list | grep scan`
  
- **If you get "Failed to spin map subscription":**
  - The `map_saver_cli` tool has QoS mismatch issues. Use Method 1 (the script) instead.
  
- **If script says "Missing required dependencies":**
  - Install: `apt-get update && apt-get install -y python3-numpy python3-pil python3-yaml`

## Docker Workflow (optional)
- Allow the container to use your display (WSLg/X11):
  ```bash
  xhost +si:localuser:root
  ```
- Start the software-rendered stack (stable under WSLg):
  ```bash
  docker compose up --build ackermann_sim
  ```
- Clean up orphan containers (if you see warnings about removed services):
  ```bash
  docker compose down --remove-orphans
  ```
- Open a shell inside the container:
  ```bash
  docker compose exec ackermann_sim bash
  ```
- Build with symlinks so edits show up immediately and source the environment every shell:
  ```bash
  colcon build --symlink-install
  source /opt/ros/jazzy/setup.bash
  source /root/colcon_ws/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export ROS_DISABLE_SHARED_MEMORY=1
  source /root/colcon_ws/install/setup.bash
  ```
- Use the same launch/teleop/SLAM/Nav2 commands as in the local section (paths already set inside the container).

## Updating the Bari World with `osm_city_pipeline`

This stack now relies exclusively on the meshes committed under `saye_description`.
To refresh the city model:

1. `cd ~/ackermann_sim/src/osm_city_pipeline` and run
   `./scripts/osm-city reset --osm-file maps/bari.osm` followed by
   `./scripts/generate_enhanced_world.sh maps/bari.osm`. The script copies the
   new `bari_world.sdf` and `models/bari_3d/` directly into
   `../ackermann-vehicle-gzsim-ros2/saye_description/`.
2. Rebuild inside the Ackermann container:
   ```bash
   docker compose exec ackermann_sim bash -lc '
     source /opt/ros/jazzy/setup.bash &&
     cd /root/colcon_ws &&
     colcon build --symlink-install
   '
   ```
3. Relaunch `ros2 launch saye_bringup saye_spawn.launch.py gui:=true` (or run
   `gz sim /root/colcon_ws/install/saye_description/share/saye_description/worlds/bari_world.sdf`
   to preview).

No dependency on `map_osm_converter` remains; all assets come straight from the
pipeline.
