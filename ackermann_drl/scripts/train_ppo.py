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
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.vec_env import DummyVecEnv
import torch

# Add package path
package_path = Path(__file__).parent.parent
sys.path.insert(0, str(package_path))

from ackermann_drl.envs.gym_wrapper import AckermannGymEnv


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
    
    # Wrap with Monitor for logging
    monitor_env = Monitor(env, str(log_dir), allow_early_resets=True)
    
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
    
    # Train the agent
    print("=" * 60)
    print("Starting training...")
    print("=" * 60)
    print()
    
    try:
        model.learn(
            total_timesteps=args.total_timesteps,
            callback=checkpoint_callback,
            progress_bar=True
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
