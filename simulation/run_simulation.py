#!/usr/bin/env python3
"""
Main simulation runner — connects physics engine to QEMU firmware
"""

import sys
import time
import numpy as np

from drone_physics.rigid_body import DroneDynamics
from drone_physics.motor_model import MotorModel
from sensors.imu_model import IMUModel
from bridge.qemu_serial_bridge import QEMUSerialBridge


def main():
    print("=" * 60)
    print("  FLIGHT CONTROLLER SIMULATION")
    print("=" * 60)
    
    # Initialize components
    drone = DroneDynamics()
    motors = MotorModel()
    imu = IMUModel(sample_rate=8000)
    
    # Initialize bridge
    bridge = QEMUSerialBridge(socket_path='/tmp/qemu_serial.sock')
    
    if not bridge.connect():
        print("[SIM] QEMU not running. Starting standalone simulation...")
        # Run without QEMU (just physics for testing)
        run_standalone(drone, motors, imu)
        return
    
    bridge.start()
    print("[SIM] Simulation running at 8kHz...")
    print("[SIM] Press Ctrl+C to stop")
    
    # Simulation loop
    dt = 1.0 / 8000.0  # 125 microseconds
    loop_count = 0
    
    try:
        while True:
            loop_start = time.time()
            
            # Get motor commands from firmware
            motor_pwm = bridge.get_motor_commands()
            
            # Update drone physics
            drone.update(motor_pwm, dt)
            
            # Get ground truth
            state = drone.get_state()
            
            # Generate sensor readings
            true_gyro = state['angular_velocity'] * 180.0 / np.pi  # rad/s to deg/s
            true_accel = state['velocity']  # Simplified — should include gravity
            
            # Add gravity in body frame
            R = drone._quat_to_matrix(state['quaternion'])
            gravity_body = R.T @ np.array([0, 0, -9.81])
            
            noisy_gyro, noisy_accel = imu.generate_reading(true_gyro, true_accel + gravity_body)
            
            # Send to firmware
            bridge.set_sensor_data(noisy_gyro, noisy_accel)
            
            # Print status every 100ms
            if loop_count % 800 == 0:
                euler = state['attitude'] * 180.0 / np.pi
                print(f"[SIM] t={loop_count*dt:.2f}s | "
                      f"Roll={euler[0]:6.1f}° Pitch={euler[1]:6.1f}° "
                      f"Alt={state['position'][2]:5.1f}m | "
                      f"Motors={motor_pwm}")
            
            loop_count += 1
            
            # Maintain 8kHz timing
            elapsed = time.time() - loop_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
                
    except KeyboardInterrupt:
        print("\n[SIM] Simulation stopped")
    finally:
        bridge.stop()
        
    print(f"[SIM] Total iterations: {loop_count}")
    print(f"[SIM] Simulated time: {loop_count * dt:.2f} seconds")


def run_standalone(drone, motors, imu):
    """Run simulation without QEMU (testing only)"""
    print("[SIM] Standalone mode (no firmware connected)")
    print("[SIM] Motors set to hover (1500 PWM)")
    
    dt = 1.0 / 8000.0
    
    try:
        while True:
            # Hover command
            motor_pwm = np.array([1500, 1500, 1500, 1500])
            
            drone.update(motor_pwm, dt)
            state = drone.get_state()
            
            time.sleep(dt)
            
    except KeyboardInterrupt:
        print("\n[SIM] Stopped")


if __name__ == '__main__':
    main()