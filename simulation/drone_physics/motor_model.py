"""
Motor and ESC simulation model
"""

import numpy as np


class MotorModel:
    """Simulates brushless motor + ESC behavior"""
    
    def __init__(self, num_motors=4):
        self.num_motors = num_motors
        self.motor_rpm = np.zeros(num_motors)
        self.time_constant = 0.02  # seconds
        self.max_rpm = 25000.0
        self.kv = 2000.0  # RPM per volt
        
    def update(self, pwm_values, battery_voltage, dt):
        """Update motor RPMs based on PWM commands"""
        new_rpm = np.zeros(self.num_motors)
        
        for i, pwm in enumerate(pwm_values):
            # PWM to throttle
            throttle = max(0.0, min(1.0, (pwm - 1000) / 1000.0))
            
            # Target RPM
            target_rpm = throttle * self.max_rpm
            
            # First-order lag
            alpha = dt / self.time_constant
            self.motor_rpm[i] += alpha * (target_rpm - self.motor_rpm[i])
            
            new_rpm[i] = self.motor_rpm[i]
        
        return new_rpm
    
    def rpm_to_thrust(self, rpm):
        """Convert RPM to thrust (N) using propeller model"""
        # Simplified thrust coefficient model
        thrust = 5.0e-9 * rpm**2
        return min(thrust, 10.0)  # Max 10N per motor
    
    def reset(self):
        """Reset motor states"""
        self.motor_rpm = np.zeros(self.num_motors)