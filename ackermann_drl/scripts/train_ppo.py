#!/usr/bin/env python3
"""PPO Training Script for Ackermann Vehicle DRL."""

import argparse
import os
import sys
import signal
import time
import subprocess
from pathlib import Path

import rclpy
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import CheckpointCallback, BaseCallback, CallbackList
from stable_baselines3.common.vec_env import DummyVecEnv

package_path = Path(__file__).parent.parent
sys.path.insert(0, str(package_path))

from ackermann_drl.envs.gym_wrapper import AckermannGymEnv


class RewardLogger(BaseCallback):
    """Logs reward breakdown to console and TensorBoard."""
    
    def __init__(self, verbose=1, log_interval=1):
        super().__init__(verbose)
        self.log_interval = log_interval
        self.step_count = 0
        self.keys = ['reward_progress', 'reward_goal', 'reward_delivery_on_time',
                     'penalty_battery_conservation', 'penalty_efficiency',
                     'penalty_delivery_late', 'penalty_offroad', 'penalty_collision',
                     'penalty_aggressive_change', 'penalty_time', 'penalty_path_deviation']
    
    def _on_step(self):
        self.step_count += 1
        infos = self.locals.get("infos", [])
        if not infos:
            return True
        
        sums = {k: 0.0 for k in self.keys}
        maxes = {k: 0.0 for k in self.keys}
        collisions = 0
        count = 0
        
        for info in infos:
            if isinstance(info, dict):
                for k in self.keys:
                    if k in info:
                        v = info[k]
                        sums[k] += v
                        maxes[k] = min(maxes[k], v) if k == 'penalty_collision' else max(maxes[k], v)
                if info.get('is_collision', False):
                    collisions += 1
                count += 1
        
        if count > 0:
            for k in self.keys:
                self.logger.record(f"reward/{k}", sums[k] / count)
            
            should_print = (self.step_count % self.log_interval == 0) or any(
                i.get('episode', {}).get('r') is not None for i in infos
            )
            
            if should_print and self.verbose > 0:
                first = infos[0] if infos and isinstance(infos[0], dict) else {}
                print(f"\n{'='*70}", flush=True)
                print(f"[Step {self.step_count}] REWARD BREAKDOWN (avg over {count} steps)", flush=True)
                print(f"{'='*70}", flush=True)
                for k in self.keys:
                    val = sums[k] / count
                    if 'reward' in k:
                        fmt_val = f"{val:+.6f}" if val != 0.0 else " 0.000000"
                    else:
                        fmt_val = f"{val:.6f}" if val < 0.0 else " 0.000000"
                    print(f"  {'[+]' if 'reward' in k else '[-]'} {k:25s}: {fmt_val}", flush=True)
                total = sum(sums[k]/count for k in self.keys)
                print(f"{'─'*70}")
                print(f"  TOTAL: {total:+.6f}")
                print(f"{'='*70}")
                if first:
                    print(f"  State: x={first.get('pos_x', 0):.1f}, y={first.get('pos_y', 0):.1f}, "
                          f"v={first.get('velocity', 0):.2f}, dist={first.get('distance_to_goal', 0):.2f}\n")
        
        return True


def stop_robot(env, vec_env):
    """Stop robot safely."""
    try:
        if hasattr(env, 'stop'):
            env.stop()
        if hasattr(vec_env, 'envs') and vec_env.envs:
            inner = vec_env.envs[0]
            while hasattr(inner, 'env'):
                inner = inner.env
            if hasattr(inner, 'stop'):
                inner.stop()
    except:
        pass
    
    try:
        for _ in range(3):
            subprocess.run(['ros2', 'topic', 'pub', '--once', '/cmd_vel', 'geometry_msgs/msg/Twist', 
                          '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'],
                         timeout=1.0, check=False, capture_output=True)
            time.sleep(0.2)
    except:
        pass


def main():
    if not rclpy.ok():
        rclpy.init()
    
    parser = argparse.ArgumentParser(description='Train PPO agent')
    parser.add_argument('--total-timesteps', type=int, default=100000)
    parser.add_argument('--checkpoint-interval', type=int, default=10000)
    parser.add_argument('--checkpoint-dir', type=str, default=None)
    parser.add_argument('--log-dir', type=str, default=None)
    parser.add_argument('--load-model', type=str, default=None)
    parser.add_argument('--learning-rate', type=float, default=3e-4)
    parser.add_argument('--batch-size', type=int, default=64)
    parser.add_argument('--n-steps', type=int, default=2048)
    parser.add_argument('--n-epochs', type=int, default=10)
    parser.add_argument('--gamma', type=float, default=0.99)
    parser.add_argument('--gae-lambda', type=float, default=0.95)
    parser.add_argument('--clip-range', type=float, default=0.2)
    parser.add_argument('--ent-coef', type=float, default=0.01)
    parser.add_argument('--vf-coef', type=float, default=0.5)
    parser.add_argument('--max-grad-norm', type=float, default=0.5)
    parser.add_argument('--device', type=str, default='auto')
    args = parser.parse_args()
    
    checkpoint_dir = Path(args.checkpoint_dir) if args.checkpoint_dir else package_path / 'checkpoints'
    log_dir = Path(args.log_dir) if args.log_dir else package_path / 'logs'
    checkpoint_dir.mkdir(parents=True, exist_ok=True)
    log_dir.mkdir(parents=True, exist_ok=True)
    
    def signal_handler(sig, frame):
        print("\n[!] Interrupt! Stopping car...", flush=True)
        stop_robot(env, vec_env)
        raise KeyboardInterrupt
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    env = AckermannGymEnv()
    try:
        monitor_env = Monitor(env, str(log_dir), allow_early_resets=True)
    except ImportError:
        monitor_env = env
    vec_env = DummyVecEnv([lambda: monitor_env])
    
    print("=" * 60)
    print("PPO Training for Ackermann Vehicle")
    print("=" * 60)
    print(f"Checkpoint: {checkpoint_dir}")
    print(f"Logs: {log_dir}")
    print(f"Timesteps: {args.total_timesteps}")
    print(f"Observation: {env.observation_space}")
    print(f"Action: {env.action_space}\n")
    
    if args.load_model:
        if not os.path.exists(args.load_model):
            print(f"[ERROR] Model not found: {args.load_model}")
            sys.exit(1)
        print(f"[INFO] Loading: {args.load_model}")
        model = PPO.load(args.load_model, env=vec_env, device=args.device,
                        tensorboard_log=str(log_dir / 'tensorboard'), verbose=1)
    else:
        model = PPO('MlpPolicy', vec_env, learning_rate=args.learning_rate,
                   n_steps=args.n_steps, batch_size=args.batch_size, n_epochs=args.n_epochs,
                   gamma=args.gamma, gae_lambda=args.gae_lambda, clip_range=args.clip_range,
                   ent_coef=args.ent_coef, vf_coef=args.vf_coef, max_grad_norm=args.max_grad_norm,
                   verbose=1, device=args.device, tensorboard_log=str(log_dir / 'tensorboard'))
    
    callbacks = CallbackList([
        CheckpointCallback(save_freq=args.checkpoint_interval, save_path=str(checkpoint_dir),
                          name_prefix='ppo_ackermann', save_replay_buffer=True, save_vecnormalize=True),
        RewardLogger(verbose=1)
    ])
    
    try:
        import tqdm, rich
        use_pbar = True
    except ImportError:
        use_pbar = False
    
    print("=" * 60)
    print("Starting training...")
    print("=" * 60 + "\n")
    
    try:
        model.learn(total_timesteps=args.total_timesteps, callback=callbacks, progress_bar=use_pbar)
        model.save(str(checkpoint_dir / 'ppo_ackermann_final'))
        print("[+] Training completed!")
    except KeyboardInterrupt:
        print("\n[!] Interrupted")
        stop_robot(env, vec_env)
        try:
            model.save(str(checkpoint_dir / 'ppo_ackermann_interrupted'))
        except:
            pass
    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback
        traceback.print_exc()
        try:
            model.save(str(checkpoint_dir / 'ppo_ackermann_error'))
        except:
            pass
        raise
    finally:
        print("\n" + "=" * 60)
        print("Cleanup...")
        print("=" * 60)
        stop_robot(env, vec_env)
        try:
            vec_env.close()
            env.close()
        except:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except:
                pass
        print("[+] Done")


if __name__ == '__main__':
    main()
