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
The `osm_city_pipeline` directory contains a duplicate `saye_description` package. This is ignored by adding a `.colcon_ignore` file.

**If the error persists:**
1. Verify `.colcon_ignore` exists:
   ```bash
   ls -la /root/colcon_ws/src/osm_city_pipeline/saye_description/.colcon_ignore
   ```

2. Clean and rebuild:
   ```bash
   cd /root/colcon_ws
   rm -rf build install log
   colcon build
   ```

3. Or build specific packages only:
   ```bash
   colcon build --packages-select ackermann_drl saye_bringup saye_description
   ```

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

