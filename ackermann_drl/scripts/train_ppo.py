#!/usr/bin/env python3
"""PPO Training Script for Ackermann Vehicle DRL."""

import argparse
import os
import sys
import signal
import time
import subprocess
from pathlib import Path

import yaml
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
                     'penalty_aggressive_change', 'penalty_time', 'penalty_path_deviation',
                     'penalty_acceleration', 'penalty_obstacle_proximity',
                     'penalty_lateral_accel', 'penalty_reverse',
                     'penalty_speeding', 'penalty_oneway']
    
    def _on_step(self):
        # self.num_timesteps is provided by BaseCallback and tracks the global step count
        current_step = self.num_timesteps
        infos = self.locals.get("infos", [])
        if not infos:
            return True
        
        sums = {k: 0.0 for k in self.keys}
        count = 0
        
        for info in infos:
            if isinstance(info, dict):
                for k in self.keys:
                    if k in info:
                        v = info[k]
                        sums[k] += v
                count += 1
        
        if count > 0:
            for k in self.keys:
                self.logger.record(f"reward/{k}", sums[k] / count)
            
            should_print = (current_step % self.log_interval == 0) or any(
                i.get('episode', {}).get('r') is not None for i in infos
            )
            
            if should_print and self.verbose > 0:
                first = infos[0] if infos and isinstance(infos[0], dict) else {}
                print(f"\n{'='*70}", flush=True)
                print(f"[Step {current_step}] REWARD BREAKDOWN (avg over {count} steps)", flush=True)
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


def _load_drl_config(config_path: str = None) -> dict:
    candidates = []
    if config_path:
        candidates.append(Path(config_path))
    candidates.extend([
        package_path / 'config' / 'drl_params.yaml',
        Path('/root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/ackermann_drl/config/drl_params.yaml'),
        Path('/home/studente/ackermann_sim/src/ackermann-vehicle-gzsim-ros2/ackermann_drl/config/drl_params.yaml'),
        Path('/root/colcon_ws/install/ackermann_drl/share/ackermann_drl/config/drl_params.yaml'),
        Path('/home/studente/ackermann_sim/src/ackermann-vehicle-gzsim-ros2/ackermann_drl/install/ackermann_drl/share/ackermann_drl/config/drl_params.yaml'),
    ])
    for path in candidates:
        if path and path.exists():
            try:
                with open(path, 'r') as f:
                    return yaml.safe_load(f) or {}
            except Exception:
                continue
    return {}


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
    parser.add_argument('--config', type=str, default=None)
    parser.add_argument('--total-timesteps', type=int, default=None)
    parser.add_argument('--checkpoint-interval', type=int, default=10000)
    parser.add_argument('--checkpoint-dir', type=str, default=None)
    parser.add_argument('--log-dir', type=str, default=None)
    parser.add_argument('--load-model', type=str, default=None)
    parser.add_argument('--learning-rate', type=float, default=None)
    parser.add_argument('--batch-size', type=int, default=None)
    parser.add_argument('--n-steps', type=int, default=2048)
    parser.add_argument('--n-epochs', type=int, default=None)
    parser.add_argument('--gamma', type=float, default=None)
    parser.add_argument('--gae-lambda', type=float, default=None)
    parser.add_argument('--clip-range', type=float, default=0.2)
    parser.add_argument('--ent-coef', type=float, default=0.01)
    parser.add_argument('--vf-coef', type=float, default=0.5)
    parser.add_argument('--max-grad-norm', type=float, default=0.5)
    parser.add_argument('--device', type=str, default='auto')
    args = parser.parse_args()

    config = _load_drl_config(args.config)
    if not config:
        print(f"[WARN] Config not loaded (source: {args.config if args.config else 'defaults'}). Using internal defaults.", flush=True)
    elif args.config:
        print(f"[INFO] Config loaded from {args.config}", flush=True)
    else:
        print(f"[INFO] Config loaded from discovered default path.", flush=True)

    training_cfg = config.get('drl', {}).get('training', {}) if isinstance(config, dict) else {}

    def pick(value, key, default):
        if value is not None:
            return value
        cfg_val = training_cfg.get(key)
        return cfg_val if cfg_val is not None else default

    def coerce(value, cast, default):
        try:
            return cast(value)
        except (TypeError, ValueError):
            return default

    total_timesteps = coerce(pick(args.total_timesteps, 'total_timesteps', 100000), int, 100000)
    learning_rate = coerce(pick(args.learning_rate, 'learning_rate', 3e-4), float, 3e-4)
    batch_size = coerce(pick(args.batch_size, 'batch_size', 64), int, 64)
    n_epochs = coerce(pick(args.n_epochs, 'n_epochs', 10), int, 10)
    gamma = coerce(pick(args.gamma, 'gamma', 0.99), float, 0.99)
    gae_lambda = coerce(pick(args.gae_lambda, 'gae_lambda', 0.95), float, 0.95)
    
    checkpoint_dir = Path(args.checkpoint_dir) if args.checkpoint_dir else package_path / 'checkpoints'
    log_dir = Path(args.log_dir) if args.log_dir else package_path / 'logs'
    checkpoint_dir.mkdir(parents=True, exist_ok=True)
    log_dir.mkdir(parents=True, exist_ok=True)
    
    env = None
    vec_env = None

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
    print(f"Timesteps: {total_timesteps}")
    print(f"LR: {learning_rate}, Batch: {batch_size}, Epochs: {n_epochs}, Gamma: {gamma}, GAE: {gae_lambda}")
    print(f"Observation: {env.observation_space}")
    print(f"Action: {env.action_space}\n")
    
    if args.load_model:
        if not os.path.exists(args.load_model):
            print(f"[ERROR] Model not found: {args.load_model}")
            sys.exit(1)
        print(f"[INFO] Loading: {args.load_model}")
        model = PPO.load(args.load_model, env=vec_env, device=args.device,
                        tensorboard_log=str(log_dir / 'tensorboard'), verbose=1)
        print(f"[INFO] Resuming from step: {model.num_timesteps}")
    else:
        model = PPO('MlpPolicy', vec_env, learning_rate=learning_rate,
                   n_steps=args.n_steps, batch_size=batch_size, n_epochs=n_epochs,
                   gamma=gamma, gae_lambda=gae_lambda, clip_range=args.clip_range,
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
        should_reset = args.load_model is None
        
        # Calculate remaining timesteps if resuming
        steps_to_train = total_timesteps
        if not should_reset:
            if model.num_timesteps >= total_timesteps:
                 print(f"[INFO] Model already reached {model.num_timesteps} steps (target: {total_timesteps}). Increasing target by 100k.")
                 steps_to_train = 100000
            else:
                 steps_to_train = total_timesteps - model.num_timesteps
            print(f"[INFO] Training for {steps_to_train} more steps (Total target: {total_timesteps})")

        model.learn(total_timesteps=steps_to_train, callback=callbacks, progress_bar=use_pbar, reset_num_timesteps=should_reset)
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
