"""
Virtual IMU sensor with realistic noise models
"""

import numpy as np


class IMUModel:
    """Simulates MPU6000 IMU with noise, bias, and drift"""
    
    def __init__(self, sample_rate=8000):
        self.sample_rate = sample_rate
        self.dt = 1.0 / sample_rate
        
        # Noise parameters
        self.gyro_noise_density = 0.004   # deg/s/√Hz
        self.accel_noise_density = 100.0  # µg/√Hz
        
        # Bias parameters
        self.gyro_bias = np.random.randn(3) * 0.1  # deg/s
        self.accel_bias = np.random.randn(3) * 0.01  # m/s²
        self.gyro_bias_stability = 0.01   # deg/s drift per second
        
        # Scale factor errors
        self.gyro_scale = 1.0 + np.random.randn(3) * 0.001
        self.accel_scale = 1.0 + np.random.randn(3) * 0.001
        
        # Misalignment
        self.alignment = np.eye(3) + np.random.randn(3, 3) * 0.001
        
    def generate_reading(self, true_gyro, true_accel):
        """Generate noisy IMU readings from ground truth"""
        
        # Apply scale factors
        gyro = true_gyro * self.gyro_scale
        accel = true_accel * self.accel_scale
        
        # Apply misalignment
        gyro = self.alignment @ gyro
        accel = self.alignment @ accel
        
        # Add bias
        gyro += self.gyro_bias
        accel += self.accel_bias
        
        # Add bias drift (random walk)
        self.gyro_bias += np.random.randn(3) * self.gyro_bias_stability * self.dt
        
        # Add white noise
        gyro_noise_std = self.gyro_noise_density * np.sqrt(self.sample_rate) / 100.0
        accel_noise_std = self.accel_noise_density * np.sqrt(self.sample_rate) * 1e-6
        
        gyro += np.random.randn(3) * gyro_noise_std
        accel += np.random.randn(3) * accel_noise_std
        
        # Convert gyro to rad/s for firmware
        gyro_rads = gyro * (np.pi / 180.0)
        
        return gyro_rads, accel
    
    def reset(self):
        """Reset sensor biases"""
        self.gyro_bias = np.random.randn(3) * 0.1
        self.accel_bias = np.random.randn(3) * 0.01