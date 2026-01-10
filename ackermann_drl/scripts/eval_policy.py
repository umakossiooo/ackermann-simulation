#!/usr/bin/env python3
"""Policy evaluation script for trained DRL agent.

MUST RUN INSIDE DOCKER CONTAINER.
Evaluates a trained PPO model on bari_world.sdf.

Usage:
    python3 eval_policy.py --model-path <path_to_model> [--num-episodes N]
"""

import argparse
import sys
from pathlib import Path
import rclpy
import numpy as np
from stable_baselines3 import PPO
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


def evaluate_policy(model_path: str, num_episodes: int = 10, max_steps_per_episode: int = 1000):
    """Evaluate a trained policy.
    
    Args:
        model_path: Path to the trained PPO model
        num_episodes: Number of episodes to evaluate
        max_steps_per_episode: Maximum steps per episode
        
    Returns:
        Dictionary with evaluation statistics
    """
    # Load model
    print(f"Loading model from {model_path}...")
    try:
        model = PPO.load(model_path)
        print("Model loaded successfully")
    except Exception as e:
        print(f"Failed to load model: {e}")
        raise
    
    # Create environment
    print("Creating environment...")
    env = make_env()
    vec_env = DummyVecEnv([lambda: env])
    
    # Evaluation statistics
    stats = {
        'episodes': [],
        'success_count': 0,
        'collision_count': 0,
        'battery_depleted_count': 0,
        'timeout_count': 0,
        'total_episodes': num_episodes,
        'battery_levels': [],
        'offroad_violations': [],
        'offroad_distances': [],
        'episode_rewards': [],
        'episode_lengths': []
    }
    
    print()
    print("=" * 60)
    print(f"Evaluating policy for {num_episodes} episodes")
    print("=" * 60)
    print()
    
    # Run episodes
    for episode in range(num_episodes):
        obs, info = env.reset()
        episode_reward = 0.0
        episode_length = 0
        episode_battery_levels = []
        episode_offroad_distances = []
        episode_offroad_violations = 0
        
        terminated = False
        truncated = False
        
        print(f"Episode {episode + 1}/{num_episodes}...", end=' ', flush=True)
        
        for step in range(max_steps_per_episode):
            # Get action from policy
            action, _ = model.predict(obs, deterministic=True)
            
            # Step environment
            obs, reward, terminated, truncated, info = env.step(action)
            
            episode_reward += reward
            episode_length += 1
            
            # Track battery level
            battery_level = info.get('battery')
            if battery_level is None:
                battery_level = info.get('battery_level')
            if battery_level is not None:
                episode_battery_levels.append(battery_level)
            
            # Track off-road violations (positive distance means outside road)
            road_distance = info.get('road_dist')
            if road_distance is not None and road_distance > 0.0:
                episode_offroad_violations += 1
                episode_offroad_distances.append(road_distance)
            
            # Check termination
            if terminated or truncated:
                break
        
        # Determine episode outcome
        # Check if goal was reached by looking at reward components
        goal_reward = info.get('reward_goal', 0.0)
        
        if terminated:
            if goal_reward > 0:
                # Goal reward was given, so goal was reached
                stats['success_count'] += 1
                outcome = "SUCCESS (goal reached)"
            else:
                # No goal reward, so likely collision
                stats['collision_count'] += 1
                outcome = "COLLISION"
        elif truncated:
            if info.get('battery_depleted', False):
                stats['battery_depleted_count'] += 1
                outcome = "BATTERY DEPLETED"
            else:
                stats['timeout_count'] += 1
                outcome = "TIMEOUT"
        else:
            stats['timeout_count'] += 1
            outcome = "TIMEOUT"
        
        # Store episode statistics
        episode_stats = {
            'episode': episode + 1,
            'outcome': outcome,
            'reward': episode_reward,
            'length': episode_length,
            'final_battery': episode_battery_levels[-1] if episode_battery_levels else 0.0,
            'avg_battery': np.mean(episode_battery_levels) if episode_battery_levels else 0.0,
            'offroad_violations': episode_offroad_violations,
            'total_offroad_distance': np.sum(episode_offroad_distances) if episode_offroad_distances else 0.0,
            'avg_offroad_distance': np.mean(episode_offroad_distances) if episode_offroad_distances else 0.0
        }
        
        stats['episodes'].append(episode_stats)
        stats['battery_levels'].extend(episode_battery_levels)
        stats['offroad_violations'].append(episode_offroad_violations)
        stats['offroad_distances'].extend(episode_offroad_distances)
        stats['episode_rewards'].append(episode_reward)
        stats['episode_lengths'].append(episode_length)
        
        print(f"{outcome} (reward: {episode_reward:.2f}, steps: {episode_length})")
    
    # Cleanup
    vec_env.close()
    env.close()
    
    return stats


def print_statistics(stats: dict):
    """Print evaluation statistics.
    
    Args:
        stats: Dictionary with evaluation statistics
    """
    print()
    print("=" * 60)
    print("Evaluation Statistics")
    print("=" * 60)
    print()
    
    # Success rate
    success_rate = (stats['success_count'] / stats['total_episodes']) * 100.0
    print(f"Success Rate: {success_rate:.1f}% ({stats['success_count']}/{stats['total_episodes']} episodes)")
    print()
    
    # Episode outcomes
    print("Episode Outcomes:")
    print(f"  Success (goal reached): {stats['success_count']}")
    print(f"  Collision: {stats['collision_count']}")
    print(f"  Battery depleted: {stats['battery_depleted_count']}")
    print(f"  Timeout: {stats['timeout_count']}")
    print()
    
    # Battery statistics
    if stats['battery_levels']:
        avg_battery = np.mean(stats['battery_levels'])
        min_battery = np.min(stats['battery_levels'])
        max_battery = np.max(stats['battery_levels'])
        final_batteries = [ep['final_battery'] for ep in stats['episodes']]
        avg_final_battery = np.mean(final_batteries)
        
        print("Battery Statistics:")
        print(f"  Average battery level: {avg_battery:.3f} ({avg_battery*100:.1f}%)")
        print(f"  Average final battery: {avg_final_battery:.3f} ({avg_final_battery*100:.1f}%)")
        print(f"  Minimum battery: {min_battery:.3f} ({min_battery*100:.1f}%)")
        print(f"  Maximum battery: {max_battery:.3f} ({max_battery*100:.1f}%)")
        print()
    else:
        print("Battery Statistics: No data")
        print()
    
    # Off-road violations
    total_offroad_violations = sum(stats['offroad_violations'])
    episodes_with_offroad = sum(1 for v in stats['offroad_violations'] if v > 0)
    offroad_violation_rate = (episodes_with_offroad / stats['total_episodes']) * 100.0
    
    if stats['offroad_distances']:
        total_offroad_distance = np.sum(stats['offroad_distances'])
        avg_offroad_distance = np.mean(stats['offroad_distances'])
        max_offroad_distance = np.max(stats['offroad_distances'])
        
        print("Off-Road Violations:")
        print(f"  Episodes with off-road: {episodes_with_offroad}/{stats['total_episodes']} ({offroad_violation_rate:.1f}%)")
        print(f"  Total violations: {total_offroad_violations}")
        print(f"  Total off-road distance: {total_offroad_distance:.2f} m")
        print(f"  Average off-road distance: {avg_offroad_distance:.2f} m")
        print(f"  Maximum off-road distance: {max_offroad_distance:.2f} m")
        print()
    else:
        print("Off-Road Violations: None")
        print()
    
    # Episode statistics
    if stats['episode_rewards']:
        avg_reward = np.mean(stats['episode_rewards'])
        min_reward = np.min(stats['episode_rewards'])
        max_reward = np.max(stats['episode_rewards'])
        avg_length = np.mean(stats['episode_lengths'])
        
        print("Episode Statistics:")
        print(f"  Average reward: {avg_reward:.2f}")
        print(f"  Min reward: {min_reward:.2f}")
        print(f"  Max reward: {max_reward:.2f}")
        print(f"  Average episode length: {avg_length:.1f} steps")
        print()
    
    print("=" * 60)


def main():
    """Main evaluation function."""
    parser = argparse.ArgumentParser(description='Evaluate trained PPO policy')
    parser.add_argument('--model-path', type=str, required=True,
                       help='Path to trained PPO model (.zip file)')
    parser.add_argument('--num-episodes', type=int, default=10,
                       help='Number of episodes to evaluate (default: 10)')
    parser.add_argument('--max-steps', type=int, default=1000,
                       help='Maximum steps per episode (default: 1000)')
    parser.add_argument('--device', type=str, default='auto',
                       help='Device to use (cpu, cuda, or auto) (default: auto)')
    
    args = parser.parse_args()
    
    # Check if model file exists
    model_path = Path(args.model_path)
    if not model_path.exists():
        print(f"Model file not found: {model_path}")
        return 1
    
    # Initialize ROS 2
    if not rclpy.ok():
        rclpy.init()
    
    try:
        # Evaluate policy
        stats = evaluate_policy(
            str(model_path),
            num_episodes=args.num_episodes,
            max_steps_per_episode=args.max_steps
        )
        
        # Print statistics
        print_statistics(stats)
        
        return 0
    
    except KeyboardInterrupt:
        print()
        print("Evaluation interrupted by user")
        return 1
    
    except Exception as e:
        print()
        print(f"Error during evaluation: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    finally:
        # Cleanup
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
