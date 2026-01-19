"""Battery model for energy consumption tracking.

Energy consumption: drop = (alpha*distance + beta*|delta_v|) * weight_factor
where alpha = distance coefficient, beta = velocity change coefficient.
"""

from typing import Optional
import numpy as np


class BatteryModel:
    """Tracks battery level [0,1] based on distance traveled and velocity changes.
    
    Energy consumption increases with distance and sudden velocity changes.
    Heavier vehicles consume more energy.
    """
    
    def __init__(self, 
                 initial_level: float = 1.0,
                 alpha: float = 0.001,
                 beta: float = 0.01,
                 idle_drain: float = 0.0005,
                 vehicle_weight: float = 1000.0):
        """Initialize battery model with energy consumption parameters."""
        self.initial_level = np.clip(initial_level, 0.0, 1.0)
        self.battery_level = self.initial_level
        self.alpha = alpha
        self.beta = beta
        self.idle_drain = idle_drain
        self.vehicle_weight = vehicle_weight
        
        self.reference_weight = 1000.0
        self.weight_factor = self.vehicle_weight / self.reference_weight
        
        self.prev_velocity: Optional[float] = None
        self.prev_position: Optional[np.ndarray] = None
        self.last_energy_drop = 0.0
    
    def update(self, 
               position: np.ndarray, 
               velocity: float, 
               dt: float = 0.1):
        """Update battery level based on distance traveled and velocity change."""
        distance = 0.0
        if self.prev_position is not None:
            pos_array = np.array(position[:2])
            prev_pos_array = np.array(self.prev_position[:2])
            distance = np.linalg.norm(pos_array - prev_pos_array)
        
        # Calculate velocity change
        delta_v = 0.0
        if self.prev_velocity is not None:
            delta_v = abs(velocity - self.prev_velocity)
        
        # Calculate energy drop: drop = (alpha*(distance) + beta*|delta_v|) * weight_factor + (idle * dt)
        # Heavier vehicles consume more energy for the same movement
        base_energy_drop = self.alpha * distance + self.beta * delta_v
        idle_energy_drop = self.idle_drain * dt
        energy_drop = (base_energy_drop * self.weight_factor) + idle_energy_drop
        self.last_energy_drop = energy_drop
        
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
        self.last_energy_drop = 0.0
    
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

    def set_vehicle_weight(self, weight: float):
        """Set vehicle weight and update weight factor.
        
        Args:
            weight: Vehicle weight in kilograms
        """
        self.vehicle_weight = max(0.1, weight)  # Minimum 0.1 kg
        self.weight_factor = self.vehicle_weight / self.reference_weight
    
    def get_vehicle_weight(self) -> float:
        """Get current vehicle weight.
        
        Returns:
            Vehicle weight in kilograms
        """
        return self.vehicle_weight
    
    def get_weight_factor(self) -> float:
        """Get current weight factor.
        
        Returns:
            Weight factor (1.0 = reference weight of 1000 kg)
        """
        return self.weight_factor
