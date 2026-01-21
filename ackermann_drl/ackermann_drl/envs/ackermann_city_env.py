import gymnasium as gym
from gymnasium import spaces
import numpy as np
import rclpy
import math
import time
import yaml
from pathlib import Path
from std_msgs.msg import Bool

from ackermann_drl.envs.modules.ros_interface import RosInterface
from ackermann_drl.envs.modules.reward_system import RewardSystem
from ackermann_drl.envs.modules.navigation import NavigationSystem
from ackermann_drl.envs.modules.delivery_manager import DeliveryManager
from ackermann_drl.utils.battery_model import BatteryModel
from ackermann_drl.utils.delivery_points import DeliveryPoints


class AckermannCityEnv(gym.Env):
    """Gym Environment for Ackermann Vehicle DRL Training."""
    metadata = {'render_modes': ['human']}

    def __init__(self):
        super().__init__()
        if not rclpy.ok():
            rclpy.init()
        
        self.ros = RosInterface()
        self.reset_pub = self.ros.create_publisher(Bool, '/reset_simulation', 1)
        
        # Load configuration
        self._load_config()
        
        dp = DeliveryPoints()
        # Pass full point objects to DeliveryManager, not just positions
        self.delivery = DeliveryManager(dp.get_all_points())
        self.navigation = NavigationSystem()
        self.reward_system = RewardSystem(self.config_data.get('drl', {}))
        self.battery = BatteryModel(vehicle_weight=1000.0)
        self.load_min, self.load_max = self.delivery.get_load_range()
        
        # Observation space
        env_config = self.config_data.get('drl', {}).get('env', {})
        obs_config = self.config_data.get('drl', {}).get('observation', {})
        robot_config = self.config_data.get('drl', {}).get('robot', {})
        self.scan_samples = int(obs_config.get('scan_samples', 180))
        if self.scan_samples <= 0:
            self.scan_samples = 180
        self.normalize_scan = bool(obs_config.get('normalize_scan', False))
        self.scan_max_range = float(obs_config.get('scan_max_range', 10.0))
        if self.scan_max_range <= 0.0:
            self.scan_max_range = 10.0
        self.use_odom = bool(obs_config.get('use_odom', True))
        self.require_road_polygons = bool(env_config.get('require_road_polygons', False))
        self.sync_sensors = bool(env_config.get('sync_sensors', True))
        self.sensor_timeout = float(env_config.get('sensor_timeout', 0.5))
        if not np.isfinite(self.sensor_timeout) or self.sensor_timeout < 0.0:
            self.sensor_timeout = 0.5
        self.sensor_poll = float(env_config.get('sensor_poll', 0.01))
        if not np.isfinite(self.sensor_poll) or self.sensor_poll <= 0.0:
            self.sensor_poll = 0.01
        restart_every = env_config.get('episodes_per_sim_restart', 0)
        try:
            restart_every = int(restart_every)
        except (TypeError, ValueError):
            restart_every = 0
        if restart_every < 0:
            restart_every = 0
        self.episodes_per_sim_restart = restart_every
        self.sim_restart_pause = float(env_config.get('sim_restart_pause', 1.0))
        if not np.isfinite(self.sim_restart_pause) or self.sim_restart_pause < 0.0:
            self.sim_restart_pause = 1.0
        if self.require_road_polygons:
            road_polys = getattr(self.navigation.roads_geometry, 'road_polygons', [])
            if not road_polys:
                raise RuntimeError(
                    "Road polygons not available. Expected road_polygons_merged.json "
                    "for curb/sidewalk detection. Ensure maps are mounted in the container."
                )
        self.obs_dim = self.scan_samples + 13  # LiDAR + 13 state features
        
        # Low-level control shaping (rate limiter + smoothing)
        control_config = env_config.get('control', {}) if isinstance(env_config, dict) else {}
        self.max_linear_accel = max(0.0, float(control_config.get('max_linear_accel', 1.5)))
        self.max_angular_accel = max(0.0, float(control_config.get('max_angular_accel', 2.0)))
        self.smoothing_tau = max(0.0, float(control_config.get('smoothing_tau', 0.2)))
        self._filtered_linear = 0.0
        self._filtered_angular = 0.0
        self._last_odom_stamp = None
        self._last_scan_stamp = None
        self._warned_sensor_timeout = False

        self._load_collision_config(env_config)
        
        self.deadline_grace = 0.0
        self.last_mission_status = None
        
        # Define Action Space based on loaded config
        # Action: [linear_velocity, angular_velocity]
        # Normalized to [-1, 1] usually, but here we used direct values in original code.
        # Ideally, we should normalize actions for PPO stability, but sticking to previous design logic if not requested.
        # However, Box limits should match the physical constraints.
        
        # Linear velocity range: [min_linear_vel, max_linear_vel]
        # Angular velocity range: [min_angular_vel, max_angular_vel]
        self.action_space = spaces.Box(
            low=np.array([self.min_linear_vel, self.min_angular_vel]), 
            high=np.array([self.max_linear_vel, self.max_angular_vel]), 
            dtype=np.float32
        )
        
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(self.obs_dim,), dtype=np.float32)
        
        # Vehicle dimensions for curb/sidewalk detection
        vehicle_width = env_config.get('vehicle_width')
        if vehicle_width is None:
            vehicle_width = robot_config.get('vehicle_width')
        if vehicle_width is None:
            vehicle_width = robot_config.get('wheel_separation', 1.0)
        try:
            vehicle_width = float(vehicle_width)
        except (TypeError, ValueError):
            vehicle_width = 1.0
        if not np.isfinite(vehicle_width) or vehicle_width <= 0.0:
            vehicle_width = 1.0
        self.vehicle_width = vehicle_width
        self.vehicle_half_width = self.vehicle_width / 2.0
        
        self.goal_threshold = 2.0
        self.safe_distance = 2.0  # threshold for obstacle proximity
        self.steps = 0
        self.episode_idx = 0
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        self.prev_vel = 0.0  # previous velocity for acceleration calculation
        self.dt = 0.1  # seconds - time step for acceleration
        self._last_step_dt = self.dt
        self._last_step_time = None
        self._last_time_source = None
        self.offroad_steps = 0
        self.collision_count = 0
        self.collision_active = False
        self.collision_event = False
        self.collision_active_steps = 0
        self.collision_active_time = 0.0
        self.collision_grace_steps_remaining = 0
        self._lidar_collision_steps = 0
        road_rules = env_config.get('road_rules', {}) if isinstance(env_config, dict) else {}
        self.speed_limit_tolerance = float(road_rules.get('speed_limit_tolerance', 0.0))
        if not np.isfinite(self.speed_limit_tolerance) or self.speed_limit_tolerance < 0.0:
            self.speed_limit_tolerance = 0.0
        self.oneway_min_speed = float(road_rules.get('oneway_min_speed', 0.2))
        if not np.isfinite(self.oneway_min_speed) or self.oneway_min_speed < 0.0:
            self.oneway_min_speed = 0.2
        heading_tol_deg = float(road_rules.get('oneway_heading_tolerance_deg', 90.0))
        if not np.isfinite(heading_tol_deg) or heading_tol_deg <= 0.0:
            heading_tol_deg = 90.0
        self.oneway_heading_tolerance = math.radians(heading_tol_deg)
        self.spawn_pose = {
            'x': 5.55,
            'y': -94.69,
            'z': 0.35,
            'yaw': -1.5064,
        }

    def _load_config(self):
        """Load DRL parameters from YAML file."""
        self.config_data = {}
        # Try to find config file
        package_root = Path(__file__).resolve().parents[2]
        config_path = package_root / 'config' / 'drl_params.yaml'
        
        # Fallback paths
        if not config_path.exists():
            candidates = [
                Path('/root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/ackermann_drl/config/drl_params.yaml'),
                Path('/home/studente/ackermann_sim/src/ackermann-vehicle-gzsim-ros2/ackermann_drl/config/drl_params.yaml'),
                Path('/root/colcon_ws/install/ackermann_drl/share/ackermann_drl/config/drl_params.yaml'),
                Path('/home/studente/ackermann_sim/src/ackermann-vehicle-gzsim-ros2/ackermann_drl/install/ackermann_drl/share/ackermann_drl/config/drl_params.yaml')
            ]
            for c in candidates:
                if c.exists():
                    config_path = c
                    break
        
        if config_path.exists():
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f) or {}
                if not isinstance(config, dict):
                    config = {}
                self.config_data = config
                drl_config = config.get('drl', {})
                env_config = drl_config.get('env', {})
                action_config = drl_config.get('action', {})
                
                self.max_episode_steps = env_config.get('max_episode_steps', 2000)
                self.max_linear_vel = action_config.get('max_linear_velocity', 2.0)
                self.min_linear_vel = action_config.get('min_linear_velocity', -2.0)
                self.max_angular_vel = action_config.get('max_angular_velocity', 1.0)
                self.min_angular_vel = action_config.get('min_angular_velocity', -1.0)
                print(f"[AckermannCityEnv] Loaded config from {config_path}")
        else:
            print("[AckermannCityEnv] Warning: drl_params.yaml not found. Using defaults.")
            self.max_episode_steps = 2000
            self.max_linear_vel = 3.0
            self.min_linear_vel = -1.0
            self.max_angular_vel = 0.5
            self.min_angular_vel = -0.5

    def _load_collision_config(self, env_config):
        collision_config = env_config.get('collision', {}) if isinstance(env_config, dict) else {}
        self.collision_threshold = self._coerce_float(
            collision_config.get('lidar_threshold', 0.8),
            0.8,
            min_value=0.05
        )
        self.lidar_collision_patience = self._coerce_int(
            collision_config.get('lidar_patience_steps', 1),
            1,
            min_value=1
        )
        self.max_collisions = self._coerce_int(
            collision_config.get('max_collisions', 1),
            1,
            min_value=0
        )
        self.collision_grace_steps = self._coerce_int(
            collision_config.get('post_event_grace_steps', 0),
            0,
            min_value=0
        )
        self.collision_stuck_time = self._coerce_float(
            collision_config.get('stuck_time', 0.0),
            0.0,
            min_value=0.0
        )
        self.collision_stuck_steps = self._coerce_int(
            collision_config.get('stuck_steps', 0),
            0,
            min_value=0
        )
        self.offroad_patience = self._coerce_int(
            collision_config.get('offroad_patience_steps', 1),
            1,
            min_value=1
        )
        self.offroad_buffer = self._coerce_float(
            collision_config.get('offroad_buffer_m', 0.0),
            0.0,
            min_value=0.0
        )

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self._finalize_previous_episode()
        self._reset_episode_state()
        did_restart = self._maybe_restart_ros_interface()
        self.delivery.reset()
        self.battery.reset()
        self.last_mission_status = self.delivery.get_mission_status()
        
        if self.spawn_pose is None:
            self._capture_spawn_pose()
        self.reset_pub.publish(Bool(data=True))
        self.ros.reset_simulation(initial_pose=self.spawn_pose)
        time.sleep(0.5)
        wait_timeout = 10.0 if did_restart else 5.0
        self._wait_for_sensors(timeout=wait_timeout)
        odom = self.ros.get_odom()
        scan = self.ros.get_scan()
        self._last_odom_stamp = self._stamp_from_msg(odom)
        self._last_scan_stamp = self._stamp_from_msg(scan)
        
        pos = self._get_position(odom)
        goal = self.delivery.get_current_goal()

        initial_dist = np.linalg.norm(np.array(pos[:2]) - np.array(goal[:2]))
        self.reward_system.reset(initial_dist)
        
        info = {
            'road_dist': self.navigation.get_road_distance(pos[0], pos[1]),
            'battery': self.battery.get_battery_level(),
            'load': self.last_mission_status['load_weight'],
            'is_collision': False,
            'is_collision_lidar': False,
            'is_collision_offroad': False,
            'collision_event': False,
            'collision_count': 0,
            'collision_active': False,
            'collision_active_time': 0.0,
            'min_lidar_distance': 10.0,
            'velocity': 0.0,
            'pos_x': pos[0],
            'pos_y': pos[1],
            'distance_to_goal': initial_dist,
            'mission_elapsed': self.last_mission_status['elapsed'],
            'mission_deadline': self.last_mission_status['deadline'],
            'mission_remaining': self.last_mission_status['remaining'],
            'deadline_exceeded': False,
            'goal_success': False
        }
        
        return self._build_obs(odom=odom, scan=scan), info

    def _finalize_previous_episode(self):
        if self.steps > 0:
            self.episode_idx += 1

    def _reset_episode_state(self):
        self.steps = 0
        self.prev_vel = 0.0
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        self.offroad_steps = 0
        self.collision_count = 0
        self.collision_active = False
        self.collision_event = False
        self.collision_active_steps = 0
        self.collision_active_time = 0.0
        self.collision_grace_steps_remaining = 0
        self._lidar_collision_steps = 0
        self._warned_sensor_timeout = False
        self._last_odom_stamp = None
        self._last_scan_stamp = None
        self._last_step_dt = self.dt
        self._reset_time_tracking()
        self._reset_action_filter()

    def _should_restart_sim(self):
        """Check if ROS interface should be restarted based on episode count."""
        if self.episodes_per_sim_restart <= 0:
            return False
        # Restart when episode_idx is a multiple of episodes_per_sim_restart
        # (e.g., after episodes 50, 100, 150, ...)
        return self.episode_idx > 0 and (self.episode_idx % self.episodes_per_sim_restart) == 0

    def _maybe_restart_ros_interface(self):
        """Restart ROS interface periodically to prevent performance degradation."""
        if not self._should_restart_sim():
            return False
        print(
            f"[AckermannCityEnv] Restarting ROS interface after completing "
            f"{self.episode_idx} episodes (next episode: {self.episode_idx + 1})."
        )
        self._restart_ros_interface()
        return True

    def _restart_ros_interface(self):
        """Restart the ROS interface (no Gazebo restart)."""
        try:
            # Stop the old ROS interface cleanly
            self.ros.stop()
        except Exception as e:
            print(f"[AckermannCityEnv] Warning: Error stopping ROS interface: {e}")
        # Ensure rclpy is initialized
        if not rclpy.ok():
            rclpy.init()
        # Create new ROS interface
        self.ros = RosInterface()
        self.reset_pub = self.ros.create_publisher(Bool, '/reset_simulation', 1)
        # Pause to allow ROS interface to initialize (subscribers, publishers, etc.)
        if self.sim_restart_pause > 0.0:
            time.sleep(self.sim_restart_pause)

    def step(self, action):
        """Execute one step: apply action, get sensors, compute reward."""
        self.steps += 1
        
        action = np.asarray(action, dtype=np.float32).flatten()
        if len(action) < 2:
            action = np.pad(action, (0, 2 - len(action)), 'constant', constant_values=0.0)
        action = np.clip(action, self.action_space.low, self.action_space.high)

        control_dt = self._last_step_dt if np.isfinite(self._last_step_dt) and self._last_step_dt > 0.0 else self.dt
        pre_odom = self.ros.get_odom()
        pre_scan = self.ros.get_scan()
        prev_odom_stamp = self._stamp_from_msg(pre_odom) or self._last_odom_stamp
        prev_scan_stamp = self._stamp_from_msg(pre_scan) or self._last_scan_stamp

        self.current_linear_vel, self.current_angular_vel = self._apply_action_smoothing(
            float(action[0]), float(action[1]), control_dt
        )
        self.ros.publish_cmd_vel(self.current_linear_vel, self.current_angular_vel)

        odom, scan = self._wait_for_new_data(prev_odom_stamp, prev_scan_stamp)
        if odom is None:
            odom = pre_odom
        if scan is None:
            scan = pre_scan

        pos = self._get_position(odom)
        if odom:
            linear_x = float(odom.twist.twist.linear.x)
            linear_y = float(odom.twist.twist.linear.y)
            vel = math.sqrt(linear_x ** 2 + linear_y ** 2)
        else:
            linear_x = 0.0
            linear_y = 0.0
            vel = 0.0
        yaw = self._get_yaw_from_odom(odom) if odom else 0.0
        if not np.isfinite(yaw):
            yaw = 0.0

        step_dt = self._get_step_dt(odom)
        self._last_step_dt = step_dt
        self._last_odom_stamp = self._stamp_from_msg(odom)
        self._last_scan_stamp = self._stamp_from_msg(scan)

        # Calculate physical acceleration (m/s^2)
        if not np.isfinite(vel):
            vel = 0.0
        if not np.isfinite(self.prev_vel):
            self.prev_vel = 0.0
        
        acceleration = (vel - self.prev_vel) / step_dt if step_dt > 0.0 else 0.0
        acceleration = np.clip(acceleration, -20.0, 20.0)  # clamp to ±20 m/s^2
        self.prev_vel = vel
        
        self.delivery.advance_time(step_dt)
        mission_status = self.delivery.get_mission_status()
        self.last_mission_status = mission_status
        load = mission_status['load_weight']
        self.battery.set_vehicle_weight(1000.0 + load)
        self.battery.update(np.array([pos[0], pos[1]]), vel, dt=step_dt)
        
        lidar_collision_raw, min_lidar = self._check_lidar(scan)
        if lidar_collision_raw:
            self._lidar_collision_steps += 1
        else:
            self._lidar_collision_steps = 0
        lidar_collision = (
            lidar_collision_raw
            and self._lidar_collision_steps >= self.lidar_collision_patience
        )
        road_speed_limit, oneway_dir, road_heading, _ = self.navigation.get_road_rules(pos[0], pos[1])
        speed_excess = 0.0
        if road_speed_limit is not None and np.isfinite(road_speed_limit) and road_speed_limit > 0.0:
            limit = road_speed_limit + self.speed_limit_tolerance
            speed_excess = max(0.0, abs(linear_x) - limit)
        oneway_violation = 0.0
        if oneway_dir != 0 and road_heading is not None and np.isfinite(road_heading):
            if abs(linear_x) > self.oneway_min_speed:
                motion_heading = yaw if linear_x >= 0.0 else yaw + math.pi
                heading_error = self._normalize_angle(motion_heading - road_heading)
                abs_err = abs(heading_error)
                if abs_err > self.oneway_heading_tolerance:
                    denom = max(1e-3, math.pi - self.oneway_heading_tolerance)
                    oneway_violation = min(1.0, (abs_err - self.oneway_heading_tolerance) / denom)
        _, _, road_dist_edge, _ = self.navigation.get_road_info(pos[0], pos[1])
        
        # Road boundary handling:
        # - "offroad_collision" ends the episode when the vehicle body crosses the road edge.
        # - "road_violation_dist" provides a smooth penalty near the edge and outside.
        # We consider "offroad collision" if the vehicle body is outside the road (hitting the curb).
        # road_dist_edge is: dist_from_center - (road_width / 2.0)
        # Positive means center is outside. Negative means center is inside.
        # Collision occurs if: center is outside OR (center is inside but side of car hits edge)
        # Side of car hits edge if: dist_from_center + car_half_width > road_width / 2.0
        # This is equivalent to: dist_edge > -self.vehicle_half_width
        
        # However, we want to be slightly lenient to avoid false positives on the exact boundary,
        # but strict enough to catch the curb.
        # If the car is hitting the banqueta (curb), it's likely at the edge.
        # Positive buffer shrinks the allowed road area to trigger earlier.
        offroad_margin = -self.vehicle_half_width - self.offroad_buffer
        offroad_collision = road_dist_edge > offroad_margin
        if road_dist_edge > 0.0:
            self.offroad_steps += 1
        else:
            self.offroad_steps = 0
        if self.offroad_steps >= self.offroad_patience:
            offroad_collision = True
        
        collision_active = lidar_collision or offroad_collision
        collision_event, collision_terminal = self._update_collision_state(collision_active, step_dt)
        
        # Calculate obstacle proximity (LiDAR only now, road edge handled by penalty_offroad logic below)
        # Considers both physical obstacles (LiDAR)
        if not np.isfinite(min_lidar) or min_lidar < 0:
            min_lidar = self.safe_distance
        
        # LiDAR proximity: normalized [0, 1] where 1 = very close
        if min_lidar < self.safe_distance:
            obstacle_proximity = (self.safe_distance - min_lidar) / self.safe_distance
        else:
            obstacle_proximity = 0.0
        obstacle_proximity = np.clip(obstacle_proximity, 0.0, 1.0)
        
        # Road edge proximity / off-road penalty logic (single scalar for reward shaping)
        # dist_from_edge_contact > 0 means the vehicle body is inside the road.
        dist_from_edge_contact = (-road_dist_edge) - self.vehicle_half_width
        edge_warning_dist = 0.5
        if not np.isfinite(dist_from_edge_contact):
            road_violation_dist = 0.0
        elif dist_from_edge_contact >= edge_warning_dist:
            road_violation_dist = 0.0
        elif dist_from_edge_contact >= 0.0:
            # Smooth ramp near the edge (kept in meters)
            norm = (edge_warning_dist - dist_from_edge_contact) / edge_warning_dist
            road_violation_dist = (norm * norm) * edge_warning_dist
        else:
            # Outside road: base penalty + distance outside
            road_violation_dist = edge_warning_dist + (-dist_from_edge_contact)
            
        goal_reached = self.delivery.check_goal_reached(pos[:2], self.goal_threshold)
        
        goal_pos = self.delivery.get_current_goal()
        target, cte = self.navigation.get_local_target(pos[:2], goal_pos[:2])
        dist_to_goal = np.linalg.norm(np.array(pos[:2]) - np.array(goal_pos[:2]))
        
        reward, info = self.reward_system.compute_reward(
            current_dist_to_goal=dist_to_goal,
            is_collision=collision_event,  # apply collision penalty once per event
            road_dist=road_violation_dist,  # Passing violation distance (proximity to edge)
            battery_consumed=self.battery.last_energy_drop,
            battery_level=self.battery.get_battery_level(),
            mission_status=mission_status,
            current_vel=(self.current_linear_vel, self.current_angular_vel),
            goal_reached=goal_reached,
            acceleration=acceleration,
            obstacle_proximity=obstacle_proximity,
            speed_excess=speed_excess,
            oneway_violation=oneway_violation
        )
        
        battery_depleted = self.battery.is_depleted()
        max_steps_exceeded = self.steps > self.max_episode_steps
        deadline_violation = mission_status['elapsed'] > (mission_status['deadline'] + self.deadline_grace)
        terminated = collision_terminal or goal_reached
        truncated = battery_depleted or max_steps_exceeded or deadline_violation
        
        info.update({
            'road_dist': road_dist_edge,
            'battery': self.battery.get_battery_level(),
            'load': load,
            'is_collision': collision_active,
            'is_collision_lidar': lidar_collision,
            'is_collision_offroad': offroad_collision,
            'collision_terminal': collision_terminal,
            'collision_event': collision_event,
            'collision_count': self.collision_count,
            'collision_active': collision_active,
            'collision_active_time': self.collision_active_time,
            'min_lidar_distance': min_lidar,
            'velocity': vel,
            'speed_limit_mps': road_speed_limit if road_speed_limit is not None else 0.0,
            'speed_excess': speed_excess,
            'oneway_violation': oneway_violation,
            'pos_x': pos[0],
            'pos_y': pos[1],
            'distance_to_goal': dist_to_goal,
            'mission_elapsed': mission_status['elapsed'],
            'mission_deadline': mission_status['deadline'],
            'mission_remaining': mission_status['remaining'],
            'deadline_exceeded': deadline_violation,
            'battery_depleted': battery_depleted,
            'max_steps_exceeded': max_steps_exceeded,
            'goal_success': goal_reached
        })
        
        return self._build_obs(odom=odom, scan=scan), reward, terminated, truncated, info

    def _check_lidar(self, scan):
        if not scan or len(scan.ranges) == 0:
            return False, 10.0
        valid = [r for r in scan.ranges if np.isfinite(r) and r > 0]
        if not valid:
            return False, 10.0
        min_dist = min(valid)
        return min_dist < self.collision_threshold, min_dist

    def _update_collision_state(self, collision_active, step_dt):
        collision_event = False
        prev_active = self.collision_active

        if collision_active:
            self.collision_active_steps += 1
            if np.isfinite(step_dt) and step_dt > 0.0:
                self.collision_active_time += step_dt
        else:
            self.collision_active_steps = 0
            self.collision_active_time = 0.0

        if collision_active and not prev_active:
            self.collision_count += 1
            collision_event = True
            self.collision_grace_steps_remaining = self.collision_grace_steps
        elif self.collision_grace_steps_remaining > 0:
            self.collision_grace_steps_remaining -= 1

        self.collision_active = collision_active
        self.collision_event = collision_event

        should_terminate = False
        if self.max_collisions > 0 and self.collision_count >= self.max_collisions:
            if self.collision_grace_steps_remaining == 0:
                should_terminate = True
        if self.collision_stuck_steps > 0 and self.collision_active_steps >= self.collision_stuck_steps:
            should_terminate = True
        if self.collision_stuck_time > 0.0 and self.collision_active_time >= self.collision_stuck_time:
            should_terminate = True

        return collision_event, should_terminate

    def _get_position(self, odom=None):
        if odom is None:
            odom = self.ros.get_odom()
        if odom:
            p = odom.pose.pose.position
            return (p.x, p.y, p.z)
        return (0.0, 0.0, 0.0)

    def _get_yaw_from_odom(self, odom):
        q = odom.pose.pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def _coerce_int(value, default, min_value=None):
        try:
            result = int(value)
        except (TypeError, ValueError):
            return default
        if min_value is not None and result < min_value:
            return min_value
        return result

    @staticmethod
    def _coerce_float(value, default, min_value=None):
        try:
            result = float(value)
        except (TypeError, ValueError):
            return default
        if not np.isfinite(result):
            return default
        if min_value is not None and result < min_value:
            return min_value
        return result

    def _capture_spawn_pose(self, timeout=5.0):
        if self.spawn_pose is not None:
            return self.spawn_pose
        self._wait_for_sensors(timeout=timeout)
        odom = self.ros.get_odom()
        if not odom:
            return None
        p = odom.pose.pose.position
        yaw = self._get_yaw_from_odom(odom)
        self.spawn_pose = {
            'x': float(p.x),
            'y': float(p.y),
            'z': float(p.z),
            'yaw': float(yaw),
        }
        return self.spawn_pose

    def _wait_for_sensors(self, timeout=5.0):
        start = time.time()
        while (self.ros.get_odom() is None or self.ros.get_scan() is None) and (time.time() - start < timeout):
            time.sleep(0.1)

    def _reset_time_tracking(self):
        self._last_step_time = None
        self._last_time_source = None

    def _reset_action_filter(self):
        self._filtered_linear = 0.0
        self._filtered_angular = 0.0

    def _apply_action_smoothing(self, linear, angular, dt):
        dt = max(float(dt), 1e-3)
        max_dv = self.max_linear_accel * dt
        max_dw = self.max_angular_accel * dt
        linear = np.clip(linear, self._filtered_linear - max_dv, self._filtered_linear + max_dv)
        angular = np.clip(angular, self._filtered_angular - max_dw, self._filtered_angular + max_dw)
        
        if self.smoothing_tau > 1e-6:
            alpha = dt / (self.smoothing_tau + dt)
            self._filtered_linear += alpha * (linear - self._filtered_linear)
            self._filtered_angular += alpha * (angular - self._filtered_angular)
        else:
            self._filtered_linear = linear
            self._filtered_angular = angular
        
        return self._filtered_linear, self._filtered_angular

    def _get_step_dt(self, odom):
        current_time = None
        source = None

        if odom and hasattr(odom, 'header'):
            stamp = odom.header.stamp
            if stamp is not None:
                stamp_time = stamp.sec + stamp.nanosec * 1e-9
                if stamp_time > 0.0:
                    current_time = stamp_time
                    source = 'odom'

        if current_time is None:
            try:
                current_time = self.ros.get_clock().now().nanoseconds * 1e-9
                source = 'ros'
            except Exception:
                current_time = time.time()
                source = 'wall'

        if self._last_time_source != source or self._last_step_time is None:
            self._last_step_time = current_time
            self._last_time_source = source
            return self.dt

        delta = current_time - self._last_step_time
        if not np.isfinite(delta) or delta <= 0.0:
            delta = self.dt
        self._last_step_time = current_time
        return delta

    def _build_obs(self, odom=None, scan=None):
        """Build observation vector: [LiDAR(N), velocity(2), target(3), goal(3), battery, CTE, time_left, late_flag, load]."""
        obs = np.zeros(self.obs_dim, dtype=np.float32)
        
        if scan is None:
            scan = self.ros.get_scan()
        if scan and len(scan.ranges) > 0:
            ranges = np.array(scan.ranges, dtype=np.float32)
            pad_value = self.scan_max_range if self.normalize_scan else 10.0
            if len(ranges) >= self.scan_samples:
                # Uniform downsample to preserve field coverage
                idx = np.linspace(0, len(ranges) - 1, self.scan_samples).astype(int)
                ranges = ranges[idx]
            else:
                # Pad if we have fewer points
                ranges = np.pad(ranges, (0, self.scan_samples - len(ranges)), 'constant', constant_values=pad_value)
            
            # Ensure exact size in case of rounding errors in slicing
            if len(ranges) > self.scan_samples:
                ranges = ranges[:self.scan_samples]
            elif len(ranges) < self.scan_samples:
                ranges = np.pad(ranges, (0, self.scan_samples - len(ranges)), 'constant', constant_values=pad_value)
            
            ranges = np.nan_to_num(ranges, nan=pad_value, posinf=pad_value, neginf=pad_value)
            if self.normalize_scan and self.scan_max_range > 1e-6:
                ranges = np.clip(ranges, 0.0, self.scan_max_range) / self.scan_max_range
            
            obs[:self.scan_samples] = ranges
        
        # Index offset for non-LiDAR features
        idx = self.scan_samples
        
        if odom is None:
            odom = self.ros.get_odom()
        if self.use_odom and odom:
            obs[idx] = odom.twist.twist.linear.x
            obs[idx+1] = odom.twist.twist.angular.z
        
        pos = self._get_position(odom)
        goal_point = self.delivery.get_current_goal()[:2]
        target, cte = self.navigation.get_local_target(pos[:2], goal_point)
        target_point = target if target else goal_point
        dx, dy, dtheta = self._to_relative(target_point, odom)
        gdx, gdy, gdtheta = self._to_relative(goal_point, odom)
        obs[idx+2:idx+5] = [dx, dy, dtheta]
        obs[idx+5:idx+8] = [gdx, gdy, gdtheta]
        obs[idx+8] = self.battery.get_battery_level()
        obs[idx+9] = cte
        
        mission = self.last_mission_status or {
            'remaining': 0.0,
            'deadline': 1.0,
            'is_late': False,
            'load_weight': self.load_min
        }
        deadline = max(1e-3, mission.get('deadline', 1.0))
        remaining_ratio = np.clip(mission.get('remaining', 0.0) / deadline, 0.0, 1.0)
        load_ratio = 0.0
        load_span = max(1e-3, self.load_max - self.load_min)
        load_ratio = np.clip((mission.get('load_weight', self.load_min) - self.load_min) / load_span, 0.0, 1.0)
        obs[idx+10] = remaining_ratio
        obs[idx+11] = 1.0 if mission.get('is_late', False) else 0.0
        obs[idx+12] = load_ratio
        
        return obs

    def _to_relative(self, target, odom=None):
        """Convert target position from world frame to robot frame."""
        if odom is None:
            odom = self.ros.get_odom()
        if not odom:
            return 0.0, 0.0, 0.0
        
        p = odom.pose.pose.position
        q = odom.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        
        dx_w = target[0] - p.x
        dy_w = target[1] - p.y
        dx_r = dx_w * math.cos(-yaw) - dy_w * math.sin(-yaw)
        dy_r = dx_w * math.sin(-yaw) + dy_w * math.cos(-yaw)
        
        target_yaw = math.atan2(dy_w, dx_w)
        dtheta = target_yaw - yaw
        while dtheta > math.pi:
            dtheta -= 2 * math.pi
        while dtheta < -math.pi:
            dtheta += 2 * math.pi
        
        return dx_r, dy_r, dtheta

    @staticmethod
    def _stamp_from_msg(msg):
        if msg is None or not hasattr(msg, 'header'):
            return None
        stamp = msg.header.stamp
        if stamp is None:
            return None
        return stamp.sec + stamp.nanosec * 1e-9

    @staticmethod
    def _is_newer_stamp(current, previous):
        if current is None:
            return False
        if previous is None:
            return True
        return current > previous + 1e-9

    def _wait_for_new_data(self, prev_odom_stamp, prev_scan_stamp):
        if not self.sync_sensors:
            return self.ros.get_odom(), self.ros.get_scan()

        start = time.time()
        odom = self.ros.get_odom()
        scan = self.ros.get_scan()
        if prev_odom_stamp is None and prev_scan_stamp is None:
            return odom, scan

        while time.time() - start < self.sensor_timeout:
            odom_stamp = self._stamp_from_msg(odom)
            scan_stamp = self._stamp_from_msg(scan)
            if self._is_newer_stamp(odom_stamp, prev_odom_stamp) and self._is_newer_stamp(scan_stamp, prev_scan_stamp):
                return odom, scan
            time.sleep(self.sensor_poll)
            odom = self.ros.get_odom()
            scan = self.ros.get_scan()

        if not self._warned_sensor_timeout:
            print("[AckermannCityEnv] Warning: sensor sync timeout, using latest available data.")
            self._warned_sensor_timeout = True
        return odom, scan

    def close(self):
        self.ros.stop()
        super().close()
