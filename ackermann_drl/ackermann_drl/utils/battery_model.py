"""Battery model for energy consumption tracking.

MUST RUN INSIDE DOCKER CONTAINER.

Implements battery model with:
- battery_level ∈ [0,1] (normalized)
- drop = (α*(distance) + β*|Δv|) * weight_factor (energy consumption formula)
- weight_factor accounts for vehicle mass (heavier = more energy consumption)
"""

from typing import Optional
import numpy as np


class BatteryModel:
    """Battery model for tracking energy consumption.
    
    Battery level is normalized to [0,1] range.
    Energy consumption: drop = (α*(distance) + β*|Δv|) * weight_factor
    where:
    - α: distance coefficient (energy per meter)
    - β: velocity change coefficient (energy per m/s change)
    - weight_factor: vehicle weight multiplier (default: 1.0 for 1000kg reference)
    - distance: distance traveled since last update
    - Δv: change in velocity magnitude
    """
    
    def __init__(self, 
                 initial_level: float = 1.0,
                 alpha: float = 0.001,  # Energy per meter (α)
                 beta: float = 0.01,   # Energy per m/s velocity change (β)
                 vehicle_weight: float = 1000.0):  # Vehicle weight in kg
        """Initialize battery model.
        
        Args:
            initial_level: Initial battery level [0,1] (default: 1.0 = 100%)
            alpha: Distance coefficient - energy consumed per meter traveled (default: 0.001)
            beta: Velocity change coefficient - energy consumed per m/s velocity change (default: 0.01)
            vehicle_weight: Vehicle weight in kilograms (default: 1000.0 kg)
                          Used to scale energy consumption (heavier = more energy)
        """
        self.initial_level = np.clip(initial_level, 0.0, 1.0)
        self.battery_level = self.initial_level
        self.alpha = alpha
        self.beta = beta
        self.vehicle_weight = vehicle_weight
        
        # Reference weight for normalization (1000 kg = factor of 1.0)
        self.reference_weight = 1000.0
        
        # Weight factor: heavier vehicles consume more energy
        # Linear scaling: weight_factor = vehicle_weight / reference_weight
        self.weight_factor = self.vehicle_weight / self.reference_weight
        
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
        
        # Calculate energy drop: drop = (α*(distance) + β*|Δv|) * weight_factor
        # Heavier vehicles consume more energy for the same movement
        base_energy_drop = self.alpha * distance + self.beta * delta_v
        energy_drop = base_energy_drop * self.weight_factor
        
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
