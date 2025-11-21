"""Battery model for energy consumption tracking.

MUST RUN INSIDE DOCKER CONTAINER.

Implements battery model with:
- battery_level ∈ [0,1] (normalized)
- drop = α*(distance) + β*|Δv| (energy consumption formula)
"""

from typing import Optional
import numpy as np


class BatteryModel:
    """Battery model for tracking energy consumption.
    
    Battery level is normalized to [0,1] range.
    Energy consumption: drop = α*(distance) + β*|Δv|
    where:
    - α: distance coefficient (energy per meter)
    - β: velocity change coefficient (energy per m/s change)
    - distance: distance traveled since last update
    - Δv: change in velocity magnitude
    """
    
    def __init__(self, 
                 initial_level: float = 1.0,
                 alpha: float = 0.001,  # Energy per meter (α)
                 beta: float = 0.01):   # Energy per m/s velocity change (β)
        """Initialize battery model.
        
        Args:
            initial_level: Initial battery level [0,1] (default: 1.0 = 100%)
            alpha: Distance coefficient - energy consumed per meter traveled (default: 0.001)
            beta: Velocity change coefficient - energy consumed per m/s velocity change (default: 0.01)
        """
        self.initial_level = np.clip(initial_level, 0.0, 1.0)
        self.battery_level = self.initial_level
        self.alpha = alpha
        self.beta = beta
        
        # Track previous state for Δv calculation
        self.prev_velocity: Optional[float] = None
        self.prev_position: Optional[np.ndarray] = None
    
    def update(self, 
               position: np.ndarray, 
               velocity: float, 
               dt: float = 0.1):
        """Update battery level based on distance traveled and velocity change.
        
        Args:
            position: Current position [x, y] or [x, y, z] (meters)
            velocity: Current velocity magnitude (m/s)
            dt: Time step (seconds) - used for validation, not in formula
        """
        # Calculate distance traveled
        distance = 0.0
        if self.prev_position is not None:
            # Calculate Euclidean distance
            pos_array = np.array(position[:2])  # Use only x, y for 2D distance
            prev_pos_array = np.array(self.prev_position[:2])
            distance = np.linalg.norm(pos_array - prev_pos_array)
        
        # Calculate velocity change
        delta_v = 0.0
        if self.prev_velocity is not None:
            delta_v = abs(velocity - self.prev_velocity)
        
        # Calculate energy drop: drop = α*(distance) + β*|Δv|
        energy_drop = self.alpha * distance + self.beta * delta_v
        
        # Update battery level (decrease by energy drop)
        self.battery_level = np.clip(self.battery_level - energy_drop, 0.0, 1.0)
        
        # Update previous state
        self.prev_position = np.array(position)
        self.prev_velocity = velocity
    
    def reset(self):
        """Reset battery to initial level and clear previous state."""
        self.battery_level = self.initial_level
        self.prev_velocity = None
        self.prev_position = None
    
    def get_battery_level(self) -> float:
        """Get current battery level.
        
        Returns:
            Battery level in [0,1] range (0.0 = depleted, 1.0 = full)
        """
        return float(self.battery_level)
    
    def get_charge_percentage(self) -> float:
        """Get current battery charge percentage.
        
        Returns:
            Charge percentage (0-100)
        """
        return self.battery_level * 100.0
    
    def is_depleted(self) -> bool:
        """Check if battery is depleted.
        
        Returns:
            True if battery level is 0
        """
        return self.battery_level <= 0.0
    
    def set_battery_level(self, level: float):
        """Set battery level (for testing).
        
        Args:
            level: Battery level in [0,1] range
        """
        self.battery_level = np.clip(level, 0.0, 1.0)

