# Build Troubleshooting

## Duplicate Package Names

### Problem
If you see an error like:
```
ERROR:colcon:colcon build: Duplicate package names not supported:
- saye_description:
  - src/ackermann-vehicle-gzsim-ros2/saye_description
  - src/osm_city_pipeline/saye_description
```

### Solution
The `osm_city_pipeline` directory contains a duplicate `saye_description` package. 

**Recommended Fix: Use the build script**
```bash
# Inside Docker container
bash /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/build_workspace.sh
```

**Manual Fix: Build specific packages only**
```bash
cd /root/colcon_ws
colcon build --packages-select saye_msgs saye_description saye_control saye_behaviortree saye_bringup saye_localization ackermann_drl
```

This avoids the duplicate detection by only building packages from `ackermann-vehicle-gzsim-ros2`.

## Building Only DRL Package

To build only the DRL package (faster):
```bash
cd /root/colcon_ws
colcon build --packages-select ackermann_drl
```

## Full Clean Build

If you need a completely clean build:
```bash
cd /root/colcon_ws
rm -rf build install log
colcon build --symlink-install
```

