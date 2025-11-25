# Training Guide - Single-Agent DRL System

## Quick Start

## Prerequisites

1. **Docker container running**
2. **Gazebo with bari_world.sdf** (can run headless)
3. **ROS 2 topics available** (`/odom`, `/scan`, `/cmd_vel`)

## Training Steps

### Step 1: Start Docker Container

```bash
# From host machine
cd /home/studente/ackermann_sim/src/ackermann-vehicle-gzsim-ros2
docker compose up -d
docker compose exec ackermann_sim bash
```

### Step 2: Start Gazebo (REQUIRED - must run in background)

**⚠️ IMPORTANT:** Gazebo MUST be running before you start training!

**Option A: Background process (RECOMMENDED for training)**

```bash
# Start Gazebo in background (headless)
# Note: world:=bari_world.sdf is optional (it's the default)
docker compose exec -d ackermann_sim bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   source /root/colcon_ws/install/setup.bash && \
   ros2 launch saye_bringup saye_spawn.launch.py gui:=false"
```

**Option B: Separate terminal (for monitoring)**

```bash
# Terminal 1: Start Gazebo
docker compose exec ackermann_sim bash
source /opt/ros/jazzy/setup.bash
source /root/colcon_ws/install/setup.bash
# Note: world:=bari_world.sdf is optional (it's the default)
ros2 launch saye_bringup saye_spawn.launch.py gui:=false
# Keep this terminal open - Gazebo runs here
```

**Option C: With GUI (if X11 forwarding available)**

```bash
docker compose exec ackermann_sim bash
source /opt/ros/jazzy/setup.bash
source /root/colcon_ws/install/setup.bash
# Note: world:=bari_world.sdf is optional (it's the default)
ros2 launch saye_bringup saye_spawn.launch.py gui:=true
```

### Step 3: Verify ROS Topics (in training terminal)

```bash
# Enter container in NEW terminal for training
docker compose exec ackermann_sim bash
source /opt/ros/jazzy/setup.bash
source /root/colcon_ws/install/setup.bash
# Check topics are available (Gazebo must be running!)
ros2 topic list
# Should see: /cmd_vel, /odom, /scan, etc.
# Verify topics have data
ros2 topic echo /odom --once  # Should show odometry data
ros2 topic echo /scan --once  # Should show laser scan data
```

**⚠️ If topics are empty or missing, Gazebo is not running!**

### Step 4: Start Training (in the same terminal as Step 3)

```bash
# Basic training (100k steps, saves checkpoint every 10k steps)
python3 /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/ackermann_drl/scripts/train_ppo.py

# Or with custom parameters:
python3 /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/ackermann_drl/scripts/train_ppo.py \
    --total-timesteps 1000000 \
    --checkpoint-interval 50000 \
    --learning-rate 3e-4 \
    --batch-size 128 \
    --n-steps 2048 \
    --device auto
```

## Training Parameters

### Recommended Settings

**Quick Test (5-10 minutes):**

```bash
python3 ackermann_drl/scripts/train_ppo.py \
    --total-timesteps 10000 \
    --checkpoint-interval 5000 \
    --n-steps 256 \
    --batch-size 32 \
    --n-epochs 5
```

**Short Training (1-2 hours):**

```bash
python3 ackermann_drl/scripts/train_ppo.py \
    --total-timesteps 100000 \
    --checkpoint-interval 10000 \
    --n-steps 1024 \
    --batch-size 64 \
    --n-epochs 10
```

**Full Training (several hours):**

```bash
python3 ackermann_drl/scripts/train_ppo.py \
    --total-timesteps 1000000 \
    --checkpoint-interval 50000 \
    --n-steps 2048 \
    --batch-size 128 \
    --n-epochs 10
```

## Training Output

### Checkpoints

- Location: `ackermann_drl/checkpoints/`
- Format: `ppo_ackermann_<steps>_steps.zip`
- Final model: `ppo_ackermann_final.zip`

### Logs

- Location: `ackermann_drl/logs/`
- Monitor CSV: `*.monitor.csv`
- TensorBoard: `logs/tensorboard/`

### View Training Progress

**TensorBoard (if available):**

```bash
tensorboard --logdir ackermann_drl/logs/tensorboard
```

**Monitor CSV:**

```bash
# View latest training metrics
tail -f ackermann_drl/logs/*.monitor.csv
```

## Troubleshooting

### Issue: "No module named 'ackermann_drl'"

```bash
source /root/colcon_ws/install/setup.bash
cd /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2
```

### Issue: "ROS topics not found"

- Ensure Gazebo is running
- Check: `ros2 topic list`
- Verify topics: `ros2 topic echo /odom` (should show data)

### Issue: "Environment reset fails"

- Ensure `delivery_points.yaml` exists
- Check: `ls ackermann_drl/config/delivery_points.yaml`
- Verify roads geometry is loaded

### Issue: "Training is slow"

- Reduce `--n-steps` (e.g., 512 instead of 2048)
- Reduce `--batch-size` (e.g., 32 instead of 64)
- Use CPU if CUDA is causing issues: `--device cpu`

### Issue: "Out of memory"

- Reduce `--batch-size`
- Reduce `--n-steps`
- Close other applications

## Monitoring Training

**Check training status:**

```bash
watch -n 5 'ls -lh ackermann_drl/checkpoints/ | tail -5'
```

**View latest checkpoint:**

```bash
ls -lht ackermann_drl/checkpoints/ | head -5
```

**Check logs:**

```bash
tail -f ackermann_drl/logs/*.monitor.csv
```

## After Training

**Evaluate trained model:**

```bash
python3 ackermann_drl/scripts/eval_policy.py \
    --model-path ackermann_drl/checkpoints/ppo_ackermann_final.zip \
    --num-episodes 20
```

**Continue training from checkpoint:**

```bash
python3 ackermann_drl/scripts/train_ppo.py \
    --model-path ackermann_drl/checkpoints/ppo_ackermann_50000_steps.zip \
    --total-timesteps 200000
```

## Quick Validation Before Training

```bash
# Inside Docker container
bash /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/test/final_validation.sh
```

## Expected Training Time

- **Quick test (10k steps)**: 5-10 minutes
- **Short training (100k steps)**: 1-2 hours
- **Full training (1M steps)**: 8-12 hours (depends on hardware)

## Tips

1. Start with a quick test to ensure everything works.
2. Monitor the first few episodes to verify rewards make sense.
3. Save checkpoints frequently (every 10k-50k steps).
4. Use headless Gazebo for faster training.
5. Check logs regularly to monitor progress.

## Ready to Train!

Everything is set up and ready. Just start Gazebo and run the training script!

