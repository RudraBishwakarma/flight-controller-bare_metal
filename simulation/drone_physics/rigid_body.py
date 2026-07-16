"""
6-DOF Rigid Body Drone Dynamics Simulation
"""

import numpy as np
from math import sin, cos


class DroneDynamics:
    """Simple quadcopter physics model"""
    
    def __init__(self):
        # Physical parameters (5-inch quadcopter)
        self.mass = 0.7           # kg
        self.inertia = np.array([
            [0.015, 0, 0],
            [0, 0.015, 0],
            [0, 0, 0.025]
        ])                         # kg*m^2
        self.inertia_inv = np.linalg.inv(self.inertia)
        self.arm_length = 0.125    # m (5-inch)
        self.max_thrust = 8.0      # N per motor
        
        # State vector
        self.position = np.zeros(3)          # x, y, z (world frame)
        self.velocity = np.zeros(3)          # vx, vy, vz
        self.attitude = np.array([1.0, 0.0, 0.0, 0.0])  # quaternion
        self.angular_velocity = np.zeros(3)  # p, q, r (body frame)
        
        # Constants
        self.gravity = np.array([0.0, 0.0, -9.81])
        self.drag_coeff = 0.3
        self.motor_tau = 0.02     # Motor time constant (seconds)
        self.current_rpm = np.zeros(4)
    
    def motor_forces(self, pwm_values):
        """Convert PWM [1000-2000] to thrust forces"""
        forces = np.zeros(4)
        torques = np.zeros(4)
        
        for i, pwm in enumerate(pwm_values):
            # PWM to throttle fraction
            if pwm <= 1000:
                throttle = 0.0
            else:
                throttle = (pwm - 1000) / 1000.0
            
            # Throttle to RPM (max ~25000 RPM)
            target_rpm = throttle * 25000.0
            
            # First-order motor dynamics
            alpha = 0.001 / self.motor_tau  # dt=1ms
            self.current_rpm[i] += alpha * (target_rpm - self.current_rpm[i])
            
            # RPM to thrust (simplified quadratic)
            thrust = 5.0e-9 * self.current_rpm[i]**2
            thrust = min(thrust, self.max_thrust)
            
            forces[i] = thrust
            
            # Motor torque (CW/CCW alternating)
            sign = 1.0 if i % 2 == 0 else -1.0
            torques[i] = sign * thrust * 0.01
        
        return forces, torques
    
    def update(self, motor_pwm, dt):
        """Update drone state for one time step"""
        
        # Get motor forces
        forces, motor_torques = self.motor_forces(motor_pwm)
        
        # Total thrust in body frame
        total_thrust = np.sum(forces)
        force_body = np.array([0.0, 0.0, total_thrust])
        
        # Convert quaternion to rotation matrix
        R = self._quat_to_matrix(self.attitude)
        
        # Force in world frame
        force_world = R @ force_body
        
        # Add gravity
        force_world += self.mass * self.gravity
        
        # Add drag
        drag = -self.drag_coeff * self.velocity * np.abs(self.velocity)
        force_world += drag
        
        # Linear acceleration
        acceleration = force_world / self.mass
        
        # Torques from motor layout (Quad-X)
        l = self.arm_length / np.sqrt(2)
        roll_torque  = (forces[0] - forces[1] + forces[2] - forces[3]) * l
        pitch_torque = (forces[0] + forces[1] - forces[2] - forces[3]) * l
        yaw_torque   = np.sum(motor_torques)
        
        torque_body = np.array([roll_torque, pitch_torque, yaw_torque])
        
        # Gyroscopic torque
        gyro = -np.cross(self.angular_velocity, self.inertia @ self.angular_velocity)
        torque_body += gyro
        
        # Angular acceleration
        angular_accel = self.inertia_inv @ torque_body
        
        # Integrate (semi-implicit Euler)
        self.velocity += acceleration * dt
        self.position += self.velocity * dt
        self.angular_velocity += angular_accel * dt
        self.attitude = self._integrate_quaternion(self.attitude, self.angular_velocity, dt)
        self.attitude /= np.linalg.norm(self.attitude)
    
    def get_state(self):
        """Return current state for sensor simulation"""
        euler = self._quat_to_euler(self.attitude)
        return {
            'position': self.position.copy(),
            'velocity': self.velocity.copy(),
            'attitude': euler,
            'angular_velocity': self.angular_velocity.copy(),
            'quaternion': self.attitude.copy()
        }
    
    def reset(self):
        """Reset to initial state"""
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.attitude = np.array([1.0, 0.0, 0.0, 0.0])
        self.angular_velocity = np.zeros(3)
        self.current_rpm = np.zeros(4)
    
    def _quat_to_matrix(self, q):
        """Quaternion to rotation matrix"""
        w, x, y, z = q
        return np.array([
            [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
            [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
            [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]
        ])
    
    def _quat_to_euler(self, q):
        """Quaternion to Euler angles (radians)"""
        w, x, y, z = q
        roll  = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        pitch = np.arcsin(2*(w*y - z*x))
        yaw   = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        return np.array([roll, pitch, yaw])
    
    def _integrate_quaternion(self, q, omega, dt):
        """Integrate quaternion with angular velocity"""
        w, x, y, z = q
        wx, wy, wz = omega * 0.5 * dt
        return np.array([
            w - wx*x - wy*y - wz*z,
            x + wx*w + wz*y - wy*z,
            y + wy*w - wz*x + wx*z,
            z + wz*w + wy*x - wx*y
        ])