## Requirements
This repository must be run **inside the provided Docker container**. All dependencies (ROS 2 Jazzy or Humble, Nav2, RViz2, `ros-gz`, Gazebo Sim Harmonic, etc.) are preinstalled in the container. 

- [Docker](https://docs.docker.com/get-docker/) and [Docker Compose](https://docs.docker.com/compose/) are required on your host system.
- **Do not install ROS or Gazebo directly on your host.** All interaction should be through the terminal inside the running container.

## Cloning

```bash
mkdir -p ~/ackermann_sim/src
cd ~/ackermann_sim/src
git clone https://github.com/umakossiooo/ackermann-simulation.git
cd ..
```

## Repository setup on the local
- mkdir -p ~/ackermann_sim/src && cd ~/ackermann_sim/src
- git clone https://github.com/umakossiooo/ackermann-simulation.git

## Docker Workflow
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
  ```

## Ackermann Steering Vehicle Simulation (ROS 2 + Gazebo Sim Harmonic)

Minimal ROS 2/Gazebo Harmonic setup for an Ackermann car with IMU, LiDAR, cameras, SLAM, and Nav2 already wired together.

**Default map:** The Bari world (`saye_description/worlds/bari_world.sdf`) and its 2D Nav2 map (`saye_bringup/maps/bari_map.yaml`) load by default. The vehicle now spawns on Via Andrea da Bari (city center), using coordinates from the map. Override `map:=...` or the `robot_*` launch arguments only if you need a different location.

## Running Inside the Docker Container

### 1. Launch Gazebo Harmonic (with RViz bridge)
```bash
ros2 launch saye_bringup saye_spawn.launch.py gui:=true
```

### 2. Run Nav2 Navigation (with AMCL on Bari map)
```bash
ros2 launch saye_bringup navigation_bringup.launch.py
```

**Note:** The `bari_map.yaml` is loaded by default. To use a different map, override with `map:=/path/to/your_map.yaml`.

### 3. Run SLAM (slam_toolbox mapping)
```bash
ros2 launch saye_bringup slam.launch.py gui:=true
```

### 4. Run SLAM + Nav2 (mapping and planning in one)
```bash
ros2 launch saye_bringup slam_navigation.launch.py gui:=true
```

### 5. Teleoperation (optional manual driving)
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## Saving the Bari Occupancy Grid Map

**You must save the map while SLAM is running!**

#### Prerequisites

- SLAM node is running and `/map` is published:
  ```bash
  ros2 topic list | grep /map
  ros2 topic hz /map
  ```
- If `/map` is missing, move the robot and ensure SLAM is active.

#### Method 1 (Recommended): Using the Provided Script
```bash
python3 /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/saye_bringup/scripts/save_map.py \
  /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/saye_bringup/maps/bari_map
```

#### Method 2: Using the Map Saver Service
Check if the service exists:
```bash
ros2 service list | grep map_saver
```
Then call the service:
```bash
ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap \
  "{map_url: '/root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/saye_bringup/maps/bari_map'}"
```

#### Method 3: Using map_saver_cli
```bash
ros2 run nav2_map_server map_saver_cli \
  -f /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/saye_bringup/maps/bari_map
``` 

---

## Path Planning

### Dijkstra Path Planning

Run the Dijkstra path planner for autonomous navigation within road boundaries:

```bash
python3 /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/saye_bringup/scripts/dijkstra_path_planner.py
```

### A* Path Planning

Run the A* path planner for autonomous navigation (potentially faster than Dijkstra):

```bash
python3 /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/saye_bringup/scripts/astar_path_planner.py
```

**Prerequisites (both planners):**
- Gazebo simulation must be running (launch with `ros2 launch saye_bringup saye_spawn.launch.py gui:=true`)
- Map files must be available (edges.json, map.json, and optionally road_polygons_merged.json)

**Note:** The goal position can be modified in the script by editing the `goal` variable in the `__main__` section.
