## Requirements

This repository is **completely self-contained** - all map files and dependencies are included. Simply clone the `single_drl` branch and run inside the provided Docker container.

**System Requirements:**
- [Docker](https://docs.docker.com/get-docker/) and [Docker Compose](https://docs.docker.com/compose/) installed on your host system
- **Do not install ROS or Gazebo directly on your host** - all interaction is through the Docker container terminal

**What's Included:**
- All required map files (`maps/` folder with edges.json, map.json, road_polygons_merged.json, route_goals.json)
- Complete DRL training system
- ROS 2 workspace with all packages
- Docker configuration ready to use
- **No external dependencies needed** - everything works out of the box

## Cloning

**IMPORTANT:** This repository uses the `single_drl` branch which contains all the working code. Make sure to checkout this branch after cloning.

```bash
git clone https://github.com/umakossiooo/ackermann-simulation.git ackermann-vehicle-gzsim-ros2
cd ackermann-vehicle-gzsim-ros2
git checkout single_drl
```

**No external dependencies needed** - all map files (edges.json, map.json, road_polygons_merged.json, route_goals.json) are included in the `maps/` folder.

## Docker Workflow

**IMPORTANT:** All commands must be run **inside the Docker container** from `/root/colcon_ws` directory.

### Setup Steps

1. **Allow the container to use your display (WSLg/X11):**
   ```bash
   xhost +si:localuser:root
   ```

2. **Start the Docker container:**
   ```bash
   cd ackermann-vehicle-gzsim-ros2
   docker compose up --build ackermann_sim
   ```

3. **Open a shell inside the container:**
   ```bash
   docker compose exec ackermann_sim bash
   ```
   You'll be in `/root/colcon_ws` (the workspace root).

4. **Build and source the environment (inside container):**
   ```bash
   # Build workspace (first time only)
   colcon build --symlink-install
   
   # Source ROS 2 and workspace (do this every time you open a new terminal)
   source /opt/ros/jazzy/setup.bash
   source /root/colcon_ws/install/setup.bash
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   export ROS_DISABLE_SHARED_MEMORY=1
   ```

5. **Verify setup (all files accessible):**
   ```bash
   # Check maps folder (should contain 4 JSON files)
   ls src/ackermann-vehicle-gzsim-ros2/maps/
   # Should see: edges.json, map.json, road_polygons_merged.json, route_goals.json
   
   # Verify repository structure
   ls src/ackermann-vehicle-gzsim-ros2/
   # Should see: ackermann_drl/, saye_bringup/, maps/, etc.
   ```

**Important:** This repository is **completely self-contained**. All map files are included in the `maps/` folder. No external repositories or dependencies are needed.

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
# Inside container, from /root/colcon_ws
python3 src/ackermann-vehicle-gzsim-ros2/saye_bringup/scripts/save_map.py \
  src/ackermann-vehicle-gzsim-ros2/saye_bringup/maps/bari_map
```

#### Method 2: Using the Map Saver Service
Check if the service exists:
```bash
ros2 service list | grep map_saver
```
Then call the service:
```bash
# Inside container, from /root/colcon_ws
ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap \
  "{map_url: 'src/ackermann-vehicle-gzsim-ros2/saye_bringup/maps/bari_map'}"
```

#### Method 3: Using map_saver_cli
```bash
# Inside container, from /root/colcon_ws
ros2 run nav2_map_server map_saver_cli \
  -f src/ackermann-vehicle-gzsim-ros2/saye_bringup/maps/bari_map
``` 

---

## Path Planning

### Dijkstra Path Planning

Run the Dijkstra path planner for autonomous navigation within road boundaries:

```bash
# Inside container, from /root/colcon_ws
python3 src/ackermann-vehicle-gzsim-ros2/saye_bringup/scripts/path_planning/dijkstra_path_planner.py
```

### A* Path Planning

Run the A* path planner for autonomous navigation (potentially faster than Dijkstra):

```bash
# Inside container, from /root/colcon_ws
python3 src/ackermann-vehicle-gzsim-ros2/saye_bringup/scripts/path_planning/astar_path_planner.py
```

**Prerequisites (both planners):**
- Gazebo simulation must be running (launch with `ros2 launch saye_bringup saye_spawn.launch.py gui:=true`)
- Map files are automatically loaded from `src/ackermann-vehicle-gzsim-ros2/maps/` (included in repository)

**Note:** The goal position can be modified in the script by editing the `goal` variable in the `__main__` section.

### Path Visualization

To verify that both algorithms produce the same optimal path and visualize it on a static map:

```bash
# Inside container, from /root/colcon_ws
cd src/ackermann-vehicle-gzsim-ros2/saye_bringup/scripts/path_planning
source /opt/ros/jazzy/setup.bash
source /root/colcon_ws/install/setup.bash
python3 visualize_path.py
```

The visualization image will be saved in the current directory as `path_visualization.png`.

---

## DRL Training (PPO)

**Important:** Before starting, ensure the simulation is running:
```bash
ros2 launch saye_bringup saye_spawn.launch.py gui:=true
```

### 1. Start Training (New Session)

**IMPORTANT:** Run from `/root/colcon_ws` inside the container after sourcing the environment.

This starts a new agent from scratch:
```bash
# Inside container, from /root/colcon_ws
python3 src/ackermann-vehicle-gzsim-ros2/ackermann_drl/scripts/train_ppo.py
# Or with specific timesteps:
python3 src/ackermann-vehicle-gzsim-ros2/ackermann_drl/scripts/train_ppo.py --total-timesteps 100000
```

### 2. Reset Training (Delete All History)
If you want to completely restart (delete all logs and models) to begin fresh:
```bash
rm -rf src/ackermann-vehicle-gzsim-ros2/ackermann_drl/logs/*
rm -rf src/ackermann-vehicle-gzsim-ros2/ackermann_drl/checkpoints/*
```

### 3. Resume Training (From Saved Checkpoint)
Use `Ctrl+C` to pause training safely. To resume:
```bash
python3 src/ackermann-vehicle-gzsim-ros2/ackermann_drl/scripts/train_ppo.py --total-timesteps 100000 --load-model src/ackermann-vehicle-gzsim-ros2/ackermann_drl/checkpoints/ppo_ackermann_interrupted.zip
```

### 4. Monitor Training
**Method A: Terminal (Real-time)**
The training script prints a detailed "REWARD BREAKDOWN" every step. Use this to see:
- Progress Reward (getting closer to goal?)
- Collision Penalty (did it hit something?)
- Off-road Penalty (is it driving on the sidewalk?)

**Method B: TensorBoard (Graphs)**
Logs are saved in `src/ackermann-vehicle-gzsim-ros2/ackermann_drl/logs/tensorboard`.
Since the viewer isn't installed in Docker, you can:
1. Copy the `logs` folder to your host computer.
2. Run `tensorboard --logdir logs/tensorboard` on your host.

**Method C: Excel (CSV)**
A `monitor.csv` file is saved in `src/ackermann-vehicle-gzsim-ros2/ackermann_drl/logs/`. You can open this in Excel to plot the reward curve.
