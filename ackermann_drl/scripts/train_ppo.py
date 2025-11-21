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


class RewardLoggingCallback(BaseCallback):
    """Callback to log detailed reward breakdown to TensorBoard and console."""
    
    def __init__(self, verbose=0, log_interval=100):
        super().__init__(verbose)
        self.log_interval = log_interval
        self.step_count = 0
        self.reward_keys = [
            'reward_progress',
            'reward_goal',
            'penalty_offroad',
            'penalty_collision',
            'penalty_battery',
            'penalty_time'
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
                    print(f"[CALLBACK DEBUG] Found collision in step: penalty_collision={info.get('penalty_collision')}, is_collision={info.get('is_collision')}, road_dist={info.get('road_distance')}")
        
        # Log averages to TensorBoard
        if count > 0:
            for key in self.reward_keys:
                avg_value = reward_sums[key] / count
                self.logger.record(f"reward/{key}", avg_value)
            
            # Print to console periodically
            if self.step_count % self.log_interval == 0:
                # Get diagnostic info from first info dict
                first_info = infos[0] if infos and isinstance(infos[0], dict) else {}
                has_scan = first_info.get('has_scan', False)
                has_odom = first_info.get('has_odom', False)
                has_goal = first_info.get('has_goal', False)
                road_dist = first_info.get('road_distance', -1.0)
                min_lidar = first_info.get('min_lidar_distance', -1.0)
                
                print(f"\n[Step {self.step_count}] Reward Breakdown:")
                print(f"  Progress: {reward_sums['reward_progress']/count:+.4f}")
                print(f"  Goal:     {reward_sums['reward_goal']/count:+.4f}")
                print(f"  Off-road:  {reward_sums['penalty_offroad']/count:+.4f}")
                # For collision, show both average and max (since collisions are rare, average can be misleading)
                collision_avg = reward_sums['penalty_collision']/count
                collision_max = reward_maxes['penalty_collision']
                if collision_count > 0:
                    print(f"  Collision: {collision_avg:+.4f} (avg) | {collision_max:+.4f} (max) | {collision_count} collisions in batch")
                else:
                    print(f"  Collision: {collision_avg:+.4f}")
                print(f"  Battery:   {reward_sums['penalty_battery']/count:+.4f}")
                print(f"  Time:      {reward_sums['penalty_time']/count:+.4f}")
                total = sum(reward_sums[k]/count for k in self.reward_keys)
                print(f"  Total:     {total:+.4f}")
                # Get additional diagnostics
                velocity = first_info.get('velocity', -1.0) if 'velocity' in first_info else -1.0
                distance_to_goal = first_info.get('distance_to_goal', -1.0) if 'distance_to_goal' in first_info else -1.0
                is_collision = first_info.get('is_collision', False)
                is_collision_lidar = first_info.get('is_collision_lidar', False)
                is_collision_offroad = first_info.get('is_collision_offroad', False)
                collision_threshold = 0.5  # From ackermann_city_env
                offroad_collision_threshold = 0.2  # From ackermann_city_env
                print(f"  Diagnostics: scan={has_scan}, odom={has_odom}, goal={has_goal}, road_dist={road_dist:.2f}m, min_lidar={min_lidar:.2f}m")
                print(f"  Collision: is_collision={is_collision} (lidar={is_collision_lidar}, offroad={is_collision_offroad}), min_lidar={min_lidar:.2f}m (threshold={collision_threshold}m), road_dist={road_dist:.2f}m (offroad_threshold={offroad_collision_threshold}m)")
                if velocity >= 0:
                    print(f"  Car velocity: {velocity:.2f} m/s, distance_to_goal: {distance_to_goal:.2f}m\n")
                else:
                    print()
        
        return True


def make_env():
    """Create and return the environment."""
    env = AckermannGymEnv()
    return env


def main():
    """Main training function."""
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
    print(f"Total timesteps: {args.total_timesteps}")
    print(f"Checkpoint interval: {args.checkpoint_interval}")
    print()
    
    # Create environment
    print("Creating environment...")
    env = make_env()
    
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
    
    print(f"Observation space: {env.observation_space}")
    print(f"Action space: {env.action_space}")
    print()
    
    # Create PPO agent
    print("Creating PPO agent...")
    print(f"Device: {args.device}")
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
        print("✓ Training completed successfully!")
        
    except KeyboardInterrupt:
        print()
        print("Training interrupted by user")
        # Save model before exiting
        interrupted_model_path = checkpoint_dir / 'ppo_ackermann_interrupted'
        print(f"Saving interrupted model to {interrupted_model_path}...")
        model.save(str(interrupted_model_path))
        print("✓ Model saved")
    
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
        # Cleanup
        print()
        print("Cleaning up...")
        vec_env.close()
        env.close()
        
        if rclpy.ok():
            rclpy.shutdown()
        
        print("✓ Cleanup complete")


if __name__ == '__main__':
    main()
