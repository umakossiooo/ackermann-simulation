"""Battery model for energy consumption tracking."""

from typing import Optional
import numpy as np


class BatteryModel:
    """Simple battery model for tracking energy consumption."""
    
    def __init__(self, initial_charge: float = 100.0, max_charge: float = 100.0):
        """Initialize battery model.
        
        Args:
            initial_charge: Initial battery charge percentage
            max_charge: Maximum battery charge percentage
        """
        self.initial_charge = initial_charge
        self.max_charge = max_charge
        self.current_charge = initial_charge
    
    def update(self, velocity: float, angular_velocity: float, dt: float):
        """Update battery charge based on motion.
        
        Args:
            velocity: Linear velocity (m/s)
            angular_velocity: Angular velocity (rad/s)
            dt: Time step (seconds)
        """
        # Simple energy consumption model
        # Energy proportional to velocity and angular velocity
        energy_consumption = (
            abs(velocity) * 0.1 +  # Base consumption for movement
            abs(angular_velocity) * 0.05  # Additional for steering
        ) * dt
        
        self.current_charge = max(0.0, self.current_charge - energy_consumption)
    
    def reset(self):
        """Reset battery to initial charge."""
        self.current_charge = self.initial_charge
    
    def get_charge_percentage(self) -> float:
        """Get current battery charge percentage.
        
        Returns:
            Charge percentage (0-100)
        """
        return (self.current_charge / self.max_charge) * 100.0
    
    def is_depleted(self) -> bool:
        """Check if battery is depleted.
        
        Returns:
            True if battery charge is 0
        """
        return self.current_charge <= 0.0

