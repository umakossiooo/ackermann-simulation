"""Sliding Mode Control (SMC) for Ackermann vehicle path following.

MUST RUN INSIDE DOCKER CONTAINER.

Implements SMC controller for following road polylines with:
- Lateral error (distance from road)
- Heading error (angle difference)
- Sliding surface: s = e_y + lambda * e_theta
- Control law with boundary layer to reduce chattering
"""

from typing import Tuple, Optional
import numpy as np
import math


class SlidingModeController:
    """Sliding Mode Controller for path following.
    
    Uses road distance and heading error to compute steering control.
    Implements boundary layer to reduce chattering.
    """
    
    def __init__(self,
                 lambda_param: float = 1.0,
                 K_smc: float = 2.0,
                 boundary_layer: float = 0.1,
                 max_steering: float = 1.0,
                 desired_velocity: float = 2.0):
        """Initialize SMC controller.
        
        Args:
            lambda_param: Weight for heading error in sliding surface (default: 1.0)
            K_smc: Control gain (default: 2.0)
            boundary_layer: Boundary layer thickness to reduce chattering (default: 0.1)
            max_steering: Maximum steering command (rad/s) (default: 1.0)
            desired_velocity: Desired forward velocity (m/s) (default: 2.0)
        """
        self.lambda_param = lambda_param
        self.K_smc = K_smc
        self.boundary_layer = boundary_layer
        self.max_steering = max_steering
        self.desired_velocity = desired_velocity
        
        # Vehicle parameters (Ackermann model)
        self.wheelbase = 2.5  # meters (typical for small vehicle)
        self.max_steering_angle = 0.5  # radians (~30 degrees)
    
    def compute_control(self,
                       lateral_error: float,
                       heading_error: float,
                       current_velocity: float = 0.0) -> Tuple[float, float]:
        """Compute SMC control commands.
        
        Args:
            lateral_error: Distance from road (meters, positive = right of road)
            heading_error: Heading error (radians, positive = vehicle pointing right of desired)
            current_velocity: Current forward velocity (m/s) for adaptive control
        
        Returns:
            Tuple of (steering_command, velocity_command):
            - steering_command: Angular velocity (rad/s) for cmd_vel
            - velocity_command: Linear velocity (m/s) for cmd_vel
        """
        # Compute sliding surface
        s = lateral_error + self.lambda_param * heading_error
        
        # SMC control law with boundary layer
        if abs(s) > self.boundary_layer:
            # Switching control (outside boundary layer)
            steering_raw = -self.K_smc * np.sign(s)
        else:
            # Linear control (inside boundary layer) - reduces chattering
            steering_raw = -self.K_smc * s / self.boundary_layer
        
        # Saturate steering command
        steering_command = np.clip(steering_raw, -self.max_steering, self.max_steering)
        
        # Velocity control: maintain desired velocity, reduce if large errors
        if abs(lateral_error) > 1.0 or abs(heading_error) > 0.5:
            # Slow down if large errors
            velocity_command = self.desired_velocity * 0.5
        else:
            velocity_command = self.desired_velocity
        
        return steering_command, velocity_command
    
    def compute_control_with_velocity_adaptation(self,
                                                lateral_error: float,
                                                heading_error: float,
                                                current_velocity: float) -> Tuple[float, float]:
        """Compute SMC control with velocity adaptation based on errors.
        
        Args:
            lateral_error: Distance from road (meters)
            heading_error: Heading error (radians)
            current_velocity: Current forward velocity (m/s)
        
        Returns:
            Tuple of (steering_command, velocity_command)
        """
        # Compute sliding surface
        s = lateral_error + self.lambda_param * heading_error
        
        # Adaptive velocity: slower when errors are large
        error_magnitude = np.sqrt(lateral_error**2 + (self.lambda_param * heading_error)**2)
        if error_magnitude > 1.0:
            velocity_scale = max(0.3, 1.0 - 0.5 * error_magnitude)
        else:
            velocity_scale = 1.0
        
        # SMC control law
        if abs(s) > self.boundary_layer:
            steering_raw = -self.K_smc * np.sign(s)
        else:
            steering_raw = -self.K_smc * s / self.boundary_layer
        
        steering_command = np.clip(steering_raw, -self.max_steering, self.max_steering)
        velocity_command = self.desired_velocity * velocity_scale
        
        return steering_command, velocity_command
    
    def set_desired_velocity(self, velocity: float):
        """Set desired forward velocity.
        
        Args:
            velocity: Desired velocity in m/s
        """
        self.desired_velocity = max(0.0, min(velocity, 5.0))  # Clamp to reasonable range
    
    def set_parameters(self,
                      lambda_param: Optional[float] = None,
                      K_smc: Optional[float] = None,
                      boundary_layer: Optional[float] = None):
        """Update controller parameters.
        
        Args:
            lambda_param: Weight for heading error
            K_smc: Control gain
            boundary_layer: Boundary layer thickness
        """
        if lambda_param is not None:
            self.lambda_param = lambda_param
        if K_smc is not None:
            self.K_smc = K_smc
        if boundary_layer is not None:
            self.boundary_layer = boundary_layer

