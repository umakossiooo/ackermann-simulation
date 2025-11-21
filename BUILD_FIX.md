# Fix for Duplicate Package Build Error

## Problem
Colcon detects duplicate `saye_description` package in both:
- `src/ackermann-vehicle-gzsim-ros2/saye_description` (correct one)
- `src/osm_city_pipeline/saye_description` (duplicate, should be ignored)

## Solution: Build with Explicit Package Selection

### Step 1: Build Dependencies First

```bash
# Inside Docker container
cd /root/colcon_ws

# Build all packages from ackermann-vehicle-gzsim-ros2, ignoring osm_city_pipeline
colcon build \
  --packages-select saye_description saye_bringup saye_control saye_msgs saye_localization saye_behaviortree \
  --packages-ignore-regex "osm_city_pipeline.*"
```

### Step 2: Build ackermann_drl

```bash
# After dependencies are built, build ackermann_drl
colcon build --packages-select ackermann_drl
```

### Alternative: Build Everything Except Duplicate

```bash
# Build all packages, but exclude the duplicate from osm_city_pipeline
colcon build --packages-ignore-regex "osm_city_pipeline.*"
```

### Quick Fix: Use --packages-skip

```bash
# Skip the duplicate package explicitly
colcon build --packages-skip-up-to saye_description --packages-select saye_description saye_bringup saye_control saye_msgs saye_localization saye_behaviortree ackermann_drl
```

## Recommended: Build Order

```bash
# Inside Docker container
cd /root/colcon_ws

# Clean first
rm -rf build install log

# Build dependencies (from ackermann-vehicle-gzsim-ros2 only)
colcon build \
  --packages-select saye_description saye_bringup saye_control saye_msgs saye_localization saye_behaviortree

# Source the install
source install/setup.bash

# Build ackermann_drl
colcon build --packages-select ackermann_drl
```

## Why This Works

- `--packages-select` only builds specified packages
- This avoids the duplicate detection issue
- Dependencies are built in correct order

