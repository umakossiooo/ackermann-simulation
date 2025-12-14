#!/usr/bin/env python3
"""PPO training script for Ackermann vehicle DRL.

MUST RUN INSIDE DOCKER CONTAINER.
Trains a PPO agent using stable-baselines3 in bari_world.sdf.

Usage:
    python3 train_ppo.py [--total-timesteps N] [--checkpoint-interval N]
"""

import argparse
import os
import sys
import signal
from pathlib import Path
import rclpy
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import CheckpointCallback, BaseCallback
from stable_baselines3.common.vec_env import DummyVecEnv
import torch

# Add package path
package_path = Path(__file__).parent.parent
sys.path.insert(0, str(package_path))

from ackermann_drl.envs.gym_wrapper import AckermannGymEnv

# Global references for signal handler
training_env = None
training_vec_env = None


class RewardLoggingCallback(BaseCallback):
    """Callback to log detailed reward breakdown to TensorBoard and console."""
    
    def __init__(self, verbose=0, log_interval=1):
        super().__init__(verbose)
        self.log_interval = log_interval  # Print every 1 step for maximum frequency
        self.step_count = 0
        self.reward_keys = [
            'reward_progress',
            'reward_goal',
            'reward_delivery_on_time',
            'reward_battery_conservation',
            'reward_efficiency',
            'penalty_delivery_late',
            'penalty_offroad',
            'penalty_collision',
            'penalty_high_speed',
            'penalty_aggressive_change',
            'penalty_time',
            'penalty_path_deviation'
        ]
    
    def _on_step(self) -> bool:
        """Log reward breakdown after each step."""
        self.step_count += 1
        
        # Get infos from the rollout buffer
        infos = self.locals.get("infos", [])
        
        if not infos:
            return True
        
        # Aggregate reward components across all environments
        reward_sums = {key: 0.0 for key in self.reward_keys}
        reward_maxes = {key: 0.0 for key in self.reward_keys}  # Track maximum values
        collision_count = 0  # Count how many steps had collisions
        count = 0
        
        # Track episode ends to log reward breakdown even for short episodes
        episode_ended = False
        
        for info in infos:
            if isinstance(info, dict):
                for key in self.reward_keys:
                    if key in info:
                        value = info[key]
                        reward_sums[key] += value
                        # Track maximum (for collision, this will be the most negative)
                        if key == 'penalty_collision':
                            reward_maxes[key] = min(reward_maxes[key], value)  # Most negative
                        else:
                            reward_maxes[key] = max(reward_maxes[key], value)
                count += 1
                # Track collisions
                if info.get('is_collision', False):
                    collision_count += 1
                # Check if episode ended (terminated or truncated)
                if info.get('episode', {}).get('r', None) is not None:
                    episode_ended = True
        
        # Log averages to TensorBoard
        if count > 0:
            for key in self.reward_keys:
                avg_value = reward_sums[key] / count
                self.logger.record(f"reward/{key}", avg_value)
            
            # Print to console periodically OR when episode ends (for short episodes)
            should_print = (self.step_count % self.log_interval == 0) or episode_ended
            
            if should_print and self.verbose > 0:
                # Get diagnostic info from first info dict
                first_info = infos[0] if infos and isinstance(infos[0], dict) else {}
                has_scan = first_info.get('has_scan', False)
                has_odom = first_info.get('has_odom', False)
                has_goal = first_info.get('has_goal', False)
                road_dist = first_info.get('road_distance', -1.0)
                min_lidar = first_info.get('min_lidar_distance', -1.0)
                
                print(f"\n{'='*70}", flush=True)
                print(f"[Step {self.step_count}] REWARD BREAKDOWN (averaged over {count} steps)", flush=True)
                print(f"{'='*70}", flush=True)
                print(f"  [+] Progress Reward:    {reward_sums['reward_progress']/count:+.6f} (getting closer to goal)", flush=True)
                print(f"  [+] Goal Reward:        {reward_sums['reward_goal']/count:+.6f} (reaching goal)")
                print(f"  [+] Delivery On-time:   {reward_sums['reward_delivery_on_time']/count:+.6f} (delivered on time)")
                print(f"  [+] Battery Conservation: {reward_sums['reward_battery_conservation']/count:+.6f} (maintaining high battery)")
                print(f"  [+] Efficiency Reward:  {reward_sums['reward_efficiency']/count:+.6f} (progress per battery)")
                print(f"  [-] Delivery Late:      {reward_sums['penalty_delivery_late']/count:+.6f} (late delivery penalty)")
                print(f"  [-] Off-road Penalty:   {reward_sums['penalty_offroad']/count:+.6f} (driving off-road)")
                collision_sum = reward_sums['penalty_collision']
                collision_avg = collision_sum / count if count > 0 else 0.0
                collision_max = reward_maxes['penalty_collision']
                if collision_count > 0:
                    print(f"  [-] Collision Penalty:  {collision_avg:+.6f} (avg) | {collision_max:+.6f} (max) | {collision_count} collisions in {count} steps")
                else:
                    print(f"  [-] Collision Penalty:  {collision_avg:+.6f} (no collisions)")
                print(f"  [-] High Speed Penalty: {reward_sums['penalty_high_speed']/count:+.6f} (excessive speed >3.0 m/s)")
                print(f"  [-] Aggressive Change:  {reward_sums['penalty_aggressive_change']/count:+.6f} (wasteful velocity changes)")
                print(f"  [-] Time Penalty:       {reward_sums['penalty_time']/count:+.6f} (per-step penalty)")
                print(f"  [-] Path Deviation:     {reward_sums['penalty_path_deviation']/count:+.6f} (deviating from A* path)")
                total = sum(reward_sums[k]/count for k in self.reward_keys)
                print(f"{'─'*70}")
                print(f"  TOTAL REWARD:         {total:+.6f}")
                print(f"{'='*70}")
                # Get additional diagnostics
                velocity = first_info.get('velocity', -1.0) if 'velocity' in first_info else -1.0
                distance_to_goal = first_info.get('distance_to_goal', -1.0) if 'distance_to_goal' in first_info else -1.0
                is_collision = first_info.get('is_collision', False)
                is_collision_lidar = first_info.get('is_collision_lidar', False)
                is_collision_offroad = first_info.get('is_collision_offroad', False)
                scan_is_fresh = first_info.get('scan_is_fresh', False)
                odom_is_fresh = first_info.get('odom_is_fresh', False)
                scan_count = first_info.get('scan_count', 0)
                odom_count = first_info.get('odom_count', 0)
                collision_threshold = 1.5  # From ackermann_city_env (LiDAR collision threshold)
                offroad_collision_threshold = 0.5  # From ackermann_city_env (off-road/sidewalk collision threshold)
                # Get delivery time info
                delivery_elapsed = first_info.get('delivery_elapsed_time', -1.0)
                delivery_deadline = first_info.get('delivery_deadline', -1.0)
                delivery_remaining = first_info.get('delivery_time_remaining', -1.0)
                delivery_on_time = first_info.get('delivery_on_time', False)
                
                print(f"  Diagnostics: scan={has_scan} (fresh={scan_is_fresh}, count={scan_count}), odom={has_odom} (fresh={odom_is_fresh}, count={odom_count}), goal={has_goal}")
                print(f"  Sensors: road_dist={road_dist:.2f}m, min_lidar={min_lidar:.2f}m")
                if delivery_deadline > 0:
                    print(f"  Delivery Time: {delivery_elapsed:.1f}s / {delivery_deadline:.1f}s (remaining: {delivery_remaining:.1f}s) | On-time: {delivery_on_time}")
                # Highlight collisions more prominently
                if is_collision:
                    print(f"  [!] COLLISION DETECTED: is_collision={is_collision} (lidar={is_collision_lidar}, offroad={is_collision_offroad})")
                    print(f"     min_lidar={min_lidar:.2f}m (threshold={collision_threshold}m), road_dist={road_dist:.2f}m (offroad_threshold={offroad_collision_threshold}m)")
                    print(f"     Penalty applied: {collision_avg:+.4f} (max={collision_max:+.4f}, count={collision_count})")
                else:
                    print(f"  Collision: is_collision={is_collision} (lidar={is_collision_lidar}, offroad={is_collision_offroad}), min_lidar={min_lidar:.2f}m (threshold={collision_threshold}m), road_dist={road_dist:.2f}m (offroad_threshold={offroad_collision_threshold}m)")
                pos_x = first_info.get('pos_x', 0.0)
                pos_y = first_info.get('pos_y', 0.0)
                
                if velocity >= 0:
                    print(f"  Car State: x={pos_x:.1f}, y={pos_y:.1f}, v={velocity:.2f} m/s, dist_to_goal={distance_to_goal:.2f}m\n")
                else:
                    print()
        
        return True


def make_env():
    """Create and return the environment."""
    env = AckermannGymEnv()
    return env


def stop_car_safely(env):
    """Stop the car safely by calling stop() on the environment."""
    try:
        if env is None:
            return
        
        # Check if it's AckermannCityEnv directly (has stop method and cmd_vel_pub)
        if hasattr(env, 'stop') and hasattr(env, 'cmd_vel_pub'):
            env.stop()
            return
        
        # Handle DummyVecEnv - it has envs list
        if hasattr(env, 'envs') and isinstance(env.envs, list) and len(env.envs) > 0:
            # DummyVecEnv wraps environments in a list
            inner_env = env.envs[0]
            # inner_env could be Monitor -> AckermannGymEnv -> AckermannCityEnv
            if hasattr(inner_env, 'env'):
                # Monitor -> AckermannGymEnv
                if hasattr(inner_env.env, 'env'):
                    # AckermannGymEnv -> AckermannCityEnv
                    inner_env.env.env.stop()
                else:
                    # Direct AckermannGymEnv (shouldn't happen but handle it)
                    if hasattr(inner_env.env, 'stop'):
                        inner_env.env.stop()
            else:
                # Direct environment (shouldn't happen but handle it)
                if hasattr(inner_env, 'stop'):
                    inner_env.stop()
        # Handle direct environment access
        elif hasattr(env, 'env'):
            # It's wrapped (Monitor or AckermannGymEnv)
            if hasattr(env.env, 'env'):
                # Monitor -> AckermannGymEnv -> AckermannCityEnv
                env.env.env.env.stop()
            else:
                # AckermannGymEnv -> AckermannCityEnv
                if hasattr(env.env, 'env') and hasattr(env.env.env, 'stop'):
                    env.env.env.stop()
                elif hasattr(env.env, 'stop'):
                    env.env.stop()
        # Direct AckermannGymEnv or other with stop method
        elif hasattr(env, 'stop'):
            env.stop()
    except Exception as e:
        print(f"Warning: Could not stop car: {e}")


def main():
    """Main training function."""
    global training_env, training_vec_env
    
    def signal_handler(sig, frame):
        """Handle Ctrl+C to stop the car immediately."""
        print("\n\n[!] Interrupt received! Stopping car immediately...", flush=True)
        try:
            # Stop car via environment
            if training_env is not None:
                print("[SIGNAL] Stopping via training_env...", flush=True)
                stop_car_safely(training_env)
            if training_vec_env is not None:
                print("[SIGNAL] Stopping via training_vec_env...", flush=True)
                stop_car_safely(training_vec_env)
            
            # Also try direct ROS command as backup
            try:
                import subprocess
                print("[SIGNAL] Sending direct stop command via ros2 topic...", flush=True)
                subprocess.run(['ros2', 'topic', 'pub', '--once', '/cmd_vel', 'geometry_msgs/msg/Twist', 
                              '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'],
                             timeout=1.0, check=False, capture_output=True)
            except:
                pass
        except Exception as e:
            print(f"[SIGNAL] Error in signal handler: {e}", flush=True)
        print("Car stopped. Exiting...", flush=True)
        # Don't exit immediately - let the KeyboardInterrupt handler in the try block handle cleanup
        raise KeyboardInterrupt
    
    # Register signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    parser = argparse.ArgumentParser(description='Train PPO agent for Ackermann vehicle')
    parser.add_argument('--total-timesteps', type=int, default=100000,
                       help='Total number of timesteps to train (default: 100000)')
    parser.add_argument('--checkpoint-interval', type=int, default=10000,
                       help='Save checkpoint every N steps (default: 10000)')
    parser.add_argument('--checkpoint-dir', type=str, default=None,
                       help='Directory to save checkpoints (default: ackermann_drl/checkpoints/)')
    parser.add_argument('--log-dir', type=str, default=None,
                       help='Directory for training logs (default: ackermann_drl/logs/)')
    parser.add_argument('--learning-rate', type=float, default=3e-4,
                       help='Learning rate (default: 3e-4)')
    parser.add_argument('--batch-size', type=int, default=64,
                       help='Batch size (default: 64)')
    parser.add_argument('--n-steps', type=int, default=2048,
                       help='Number of steps per update (default: 2048)')
    parser.add_argument('--n-epochs', type=int, default=10,
                       help='Number of epochs per update (default: 10)')
    parser.add_argument('--gamma', type=float, default=0.99,
                       help='Discount factor (default: 0.99)')
    parser.add_argument('--gae-lambda', type=float, default=0.95,
                       help='GAE lambda (default: 0.95)')
    parser.add_argument('--clip-range', type=float, default=0.2,
                       help='PPO clip range (default: 0.2)')
    parser.add_argument('--ent-coef', type=float, default=0.01,
                       help='Entropy coefficient (default: 0.01)')
    parser.add_argument('--vf-coef', type=float, default=0.5,
                       help='Value function coefficient (default: 0.5)')
    parser.add_argument('--max-grad-norm', type=float, default=0.5,
                       help='Maximum gradient norm (default: 0.5)')
    parser.add_argument('--device', type=str, default='auto',
                       help='Device to use (cpu, cuda, or auto) (default: auto)')
    parser.add_argument('--load-model', type=str, default=None,
                       help='Path to a zip file to load and resume training from')
    
    args = parser.parse_args()
    
    # Initialize ROS 2
    if not rclpy.ok():
        rclpy.init()
    
    # Set up directories
    package_path = Path(__file__).parent.parent
    
    if args.checkpoint_dir is None:
        checkpoint_dir = package_path / 'checkpoints'
    else:
        checkpoint_dir = Path(args.checkpoint_dir)
    
    if args.log_dir is None:
        log_dir = package_path / 'logs'
    else:
        log_dir = Path(args.log_dir)
    
    # Create directories
    checkpoint_dir.mkdir(parents=True, exist_ok=True)
    log_dir.mkdir(parents=True, exist_ok=True)
    
    print("=" * 60)
    print("PPO Training for Ackermann Vehicle")
    print("=" * 60)
    print(f"Checkpoint directory: {checkpoint_dir}")
    print(f"Log directory: {log_dir}")
    print(f"TensorBoard: Logs will be saved to {log_dir / 'tensorboard'}")
    print("NOTE: To view graphs, copy the 'logs' folder to your host and run 'tensorboard --logdir logs'")
    print(f"Total timesteps: {args.total_timesteps}")
    print(f"Checkpoint interval: {args.checkpoint_interval}")
    print()
    
    # Create environment
    print("Creating environment...")
    env = make_env()
    training_env = env  # Store for signal handler
    
    # Wrap with Monitor for logging (tensorboard optional)
    try:
        monitor_env = Monitor(env, str(log_dir), allow_early_resets=True)
    except ImportError as e:
        if 'tensorboard' in str(e).lower():
            print("Warning: Tensorboard not available, continuing without logging")
            monitor_env = env
        else:
            raise
    
    # Create vectorized environment (single environment)
    vec_env = DummyVecEnv([lambda: monitor_env])
    training_vec_env = vec_env  # Store for signal handler
    
    print(f"Observation space: {env.observation_space}")
    print(f"Action space: {env.action_space}")
    print()
    
    # Create PPO agent
    print("Creating PPO agent...")
    print(f"Device: {args.device}")
    
    if args.load_model:
        print(f"[INFO] Loading existing model from: {args.load_model}")
        if not os.path.exists(args.load_model):
            print(f"[ERROR] Model file not found: {args.load_model}")
            sys.exit(1)
            
        model = PPO.load(
            args.load_model,
            env=vec_env,
            device=args.device,
            tensorboard_log=str(log_dir / 'tensorboard'),
            verbose=1,
            # Allow updating learning rate if provided? Default keeps saved one.
            # learning_rate=args.learning_rate 
        )
        print("[INFO] Model loaded successfully. Resuming training...")
    else:
        print(f"Learning rate: {args.learning_rate}")
        print(f"Batch size: {args.batch_size}")
        print(f"N steps: {args.n_steps}")
        print(f"N epochs: {args.n_epochs}")
        print()
        
        model = PPO(
            'MlpPolicy',
            vec_env,
            learning_rate=args.learning_rate,
            n_steps=args.n_steps,
            batch_size=args.batch_size,
            n_epochs=args.n_epochs,
            gamma=args.gamma,
            gae_lambda=args.gae_lambda,
            clip_range=args.clip_range,
            ent_coef=args.ent_coef,
            vf_coef=args.vf_coef,
            max_grad_norm=args.max_grad_norm,
            verbose=1,
            device=args.device,
            tensorboard_log=str(log_dir / 'tensorboard')
        )
    
    # Set up checkpoint callback
    checkpoint_callback = CheckpointCallback(
        save_freq=args.checkpoint_interval,
        save_path=str(checkpoint_dir),
        name_prefix='ppo_ackermann',
        save_replay_buffer=True,
        save_vecnormalize=True
    )
    
    # Set up reward logging callback
    reward_callback = RewardLoggingCallback(verbose=1)
    
    # Combine callbacks
    from stable_baselines3.common.callbacks import CallbackList
    callbacks = CallbackList([checkpoint_callback, reward_callback])
    
    # Train the agent
    print("=" * 60)
    print("Starting training...")
    print("=" * 60)
    print()
    
    try:
        # Use progress_bar only if tqdm/rich are available
        try:
            import tqdm
            import rich
            use_progress_bar = True
        except ImportError:
            use_progress_bar = False
        
        model.learn(
            total_timesteps=args.total_timesteps,
            callback=callbacks,
            progress_bar=use_progress_bar
        )
        
        # Save final model
        final_model_path = checkpoint_dir / 'ppo_ackermann_final'
        print()
        print(f"Saving final model to {final_model_path}...")
        model.save(str(final_model_path))
        print("[+] Training completed successfully!")
        
    except KeyboardInterrupt:
        print()
        print("=" * 60)
        print("Training interrupted by user")
        print("=" * 60)
        # Stop the car immediately - this is critical!
        print("\n[CRITICAL] Stopping car immediately...")
        try:
            stop_car_safely(env)
            stop_car_safely(vec_env)
            # Give it a moment to process
            import time
            time.sleep(0.5)
            # Try one more time to be sure
            stop_car_safely(env)
            stop_car_safely(vec_env)
            time.sleep(0.5)
        except Exception as e:
            print(f"[ERROR] Error stopping car: {e}")
            # Try direct ROS command as last resort
            try:
                import subprocess
                for _ in range(3):
                    subprocess.run(['ros2', 'topic', 'pub', '--once', '/cmd_vel', 'geometry_msgs/msg/Twist', 
                                  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'],
                                 timeout=1.0, check=False, capture_output=True)
                    time.sleep(0.2)
            except:
                pass
        print("[CRITICAL] Car stop commands sent")
        
        # Save model before exiting
        interrupted_model_path = checkpoint_dir / 'ppo_ackermann_interrupted'
        print(f"\nSaving interrupted model to {interrupted_model_path}...")
        try:
            model.save(str(interrupted_model_path))
            print("[+] Model saved")
        except Exception as e:
            print(f"Warning: Could not save model: {e}")
    
    except Exception as e:
        print()
        print(f"Error during training: {e}")
        import traceback
        traceback.print_exc()
        # Try to save model anyway
        try:
            error_model_path = checkpoint_dir / 'ppo_ackermann_error'
            model.save(str(error_model_path))
            print(f"Model saved to {error_model_path}")
        except:
            pass
        raise
    
    finally:
        # Cleanup - always stop the car (this is the last chance!)
        print()
        print("=" * 60)
        print("Final cleanup - ensuring car is stopped...")
        print("=" * 60)
        try:
            # Stop the car first - multiple attempts
            import time
            for attempt in range(3):
                print(f"[CLEANUP] Stopping car (attempt {attempt+1}/3)...")
                stop_car_safely(env)
                stop_car_safely(vec_env)
                time.sleep(0.3)
            
            # Properly close and destroy the environment
            try:
                print("[CLEANUP] Closing environment...")
                vec_env.close()
                env.close()
            except Exception as e:
                print(f"[CLEANUP] Warning during environment close: {e}")
            
            # Also try direct ROS command
            try:
                import subprocess
                print("[CLEANUP] Sending final stop command via ros2 topic...")
                for _ in range(3):
                    subprocess.run(['ros2', 'topic', 'pub', '--once', '/cmd_vel', 'geometry_msgs/msg/Twist', 
                                  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'],
                                 timeout=1.0, check=False, capture_output=True)
                    time.sleep(0.2)
            except:
                pass
        except Exception as e:
            print(f"[CLEANUP] Error during cleanup: {e}")
        
        try:
            vec_env.close()
        except:
            pass
        try:
            env.close()
        except:
            pass
        
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except:
                pass
        
        print("[+] Cleanup complete - car should be stopped")


if __name__ == '__main__':
    main()
