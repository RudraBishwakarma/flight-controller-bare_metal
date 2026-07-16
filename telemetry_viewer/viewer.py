#!/usr/bin/env python3
"""
Real-time flight controller telemetry viewer
Displays gyro, attitude, PID outputs, and motor values as live graphs
"""

import sys
import time
import struct
import threading
import numpy as np
from collections import deque

# Try importing serial, fall back to simulated data
try:
    import serial
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False
    print("[Viewer] pyserial not installed. Using simulated data.")

try:
    import matplotlib
    matplotlib.use('TkAgg')  # Use Tkinter backend
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False
    print("[Viewer] matplotlib not installed. No graphical display.")
    sys.exit(1)


# ==========================================
# DATA STORAGE (Circular Buffers)
# ==========================================

BUFFER_SIZE = 500  # Show last 500 samples

class TelemetryData:
    """Stores telemetry history for plotting"""
    
    def __init__(self):
        self.time = deque(maxlen=BUFFER_SIZE)
        
        # Gyroscope (rad/s)
        self.gyro_x = deque(maxlen=BUFFER_SIZE)
        self.gyro_y = deque(maxlen=BUFFER_SIZE)
        self.gyro_z = deque(maxlen=BUFFER_SIZE)
        
        # Attitude (degrees)
        self.roll = deque(maxlen=BUFFER_SIZE)
        self.pitch = deque(maxlen=BUFFER_SIZE)
        self.yaw = deque(maxlen=BUFFER_SIZE)
        
        # PID outputs
        self.pid_roll = deque(maxlen=BUFFER_SIZE)
        self.pid_pitch = deque(maxlen=BUFFER_SIZE)
        self.pid_yaw = deque(maxlen=BUFFER_SIZE)
        
        # Motors (PWM)
        self.motor1 = deque(maxlen=BUFFER_SIZE)
        self.motor2 = deque(maxlen=BUFFER_SIZE)
        self.motor3 = deque(maxlen=BUFFER_SIZE)
        self.motor4 = deque(maxlen=BUFFER_SIZE)
        
        # Battery
        self.battery = deque(maxlen=BUFFER_SIZE)
        
        # Status
        self.armed = deque(maxlen=BUFFER_SIZE)
        self.failsafe = deque(maxlen=BUFFER_SIZE)
        
        # Start time
        self.start_time = time.time()
    
    def add_sample(self, data):
        """Add one telemetry sample"""
        t = time.time() - self.start_time
        self.time.append(t)
        
        self.gyro_x.append(data.get('gyro_x', 0))
        self.gyro_y.append(data.get('gyro_y', 0))
        self.gyro_z.append(data.get('gyro_z', 0))
        
        self.roll.append(data.get('roll', 0))
        self.pitch.append(data.get('pitch', 0))
        self.yaw.append(data.get('yaw', 0))
        
        self.pid_roll.append(data.get('pid_roll', 0))
        self.pid_pitch.append(data.get('pid_pitch', 0))
        self.pid_yaw.append(data.get('pid_yaw', 0))
        
        self.motor1.append(data.get('motor1', 1000))
        self.motor2.append(data.get('motor2', 1000))
        self.motor3.append(data.get('motor3', 1000))
        self.motor4.append(data.get('motor4', 1000))
        
        self.battery.append(data.get('battery', 0))
        self.armed.append(1 if data.get('armed', False) else 0)
        self.failsafe.append(1 if data.get('failsafe', False) else 0)


# ==========================================
# SERIAL READER (or Simulated Data)
# ==========================================

class SerialReader:
    """Reads telemetry from serial port"""
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.running = False
        self.data = TelemetryData()
        self.lock = threading.Lock()
        self.latest = {}
    
    def connect(self):
        """Connect to serial port"""
        if not HAS_SERIAL:
            return False
        
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            print(f"[Viewer] Connected to {self.port} at {self.baudrate} baud")
            return True
        except Exception as e:
            print(f"[Viewer] Cannot connect: {e}")
            return False
    
    def start(self):
        """Start reading thread"""
        self.running = True
        thread = threading.Thread(target=self._read_loop)
        thread.daemon = True
        thread.start()
    
    def stop(self):
        """Stop reading"""
        self.running = False
        if self.ser:
            self.ser.close()
    
    def _read_loop(self):
        """Read and parse telemetry packets"""
        buffer = b''
        
        while self.running:
            if self.ser:
                try:
                    data = self.ser.read(1024)
                    if data:
                        buffer += data
                except:
                    time.sleep(0.001)
                    continue
            else:
                # Simulated data mode
                self._generate_simulated_data()
                time.sleep(0.01)  # 100Hz
                continue
            
            # Parse complete packets (simple format: 44 bytes)
            # Format: 10 floats (gyro3 + att3 + pid3 + motors4) + 1 byte status = ~44 bytes
            while len(buffer) >= 44:
                packet = buffer[:44]
                buffer = buffer[44:]
                
                try:
                    values = struct.unpack('<10fI', packet)
                    
                    sample = {
                        'gyro_x': values[0],
                        'gyro_y': values[1],
                        'gyro_z': values[2],
                        'roll': values[3],
                        'pitch': values[4],
                        'yaw': values[5],
                        'pid_roll': values[6],
                        'pid_pitch': values[7],
                        'pid_yaw': values[8],
                        'motor1': values[9] & 0xFFFF,
                        'motor2': (values[9] >> 16) & 0xFFFF,
                        'battery': 0,
                        'armed': (values[10] & 0x01) != 0,
                        'failsafe': (values[10] & 0x02) != 0
                    }
                    
                    with self.lock:
                        self.latest = sample
                        self.data.add_sample(sample)
                        
                except struct.error:
                    continue
    
    def _generate_simulated_data(self):
        """Generate fake data for testing without hardware"""
        t = time.time() - self.data.start_time
        
        sample = {
            'gyro_x': np.sin(t * 5) * 0.1,
            'gyro_y': np.cos(t * 3) * 0.05,
            'gyro_z': np.sin(t * 2) * 0.02,
            'roll': np.sin(t * 2) * 15,
            'pitch': np.cos(t * 1.5) * 10,
            'yaw': t * 5 % 360,
            'pid_roll': np.sin(t * 5) * 50,
            'pid_pitch': np.cos(t * 3) * 30,
            'pid_yaw': np.sin(t * 2) * 20,
            'motor1': 1500 + np.sin(t * 5) * 50,
            'motor2': 1500 - np.sin(t * 5) * 50,
            'motor3': 1500 + np.cos(t * 3) * 30,
            'motor4': 1500 - np.cos(t * 3) * 30,
            'battery': 16.8 - t * 0.01,
            'armed': True,
            'failsafe': False
        }
        
        with self.lock:
            self.latest = sample
            self.data.add_sample(sample)


# ==========================================
# REAL-TIME PLOTS
# ==========================================

class TelemetryViewer:
    """Real-time matplotlib visualization"""
    
    def __init__(self, reader):
        self.reader = reader
        
        # Create figure with 4 subplots
        self.fig, self.axes = plt.subplots(4, 1, figsize=(12, 10))
        self.fig.suptitle('Flight Controller Telemetry', fontsize=14)
        
        # Plot 1: Attitude (Roll, Pitch, Yaw)
        self.ax1 = self.axes[0]
        self.ax1.set_ylabel('Angle (degrees)')
        self.ax1.set_title('Attitude')
        self.ax1.grid(True, alpha=0.3)
        self.line_roll, = self.ax1.plot([], [], 'r-', label='Roll', linewidth=1)
        self.line_pitch, = self.ax1.plot([], [], 'g-', label='Pitch', linewidth=1)
        self.line_yaw, = self.ax1.plot([], [], 'b-', label='Yaw', linewidth=1)
        self.ax1.legend(loc='upper right')
        self.ax1.set_ylim(-180, 180)
        
        # Plot 2: Gyroscope
        self.ax2 = self.axes[1]
        self.ax2.set_ylabel('Rate (rad/s)')
        self.ax2.set_title('Gyroscope')
        self.ax2.grid(True, alpha=0.3)
        self.line_gx, = self.ax2.plot([], [], 'r-', label='GX', linewidth=1)
        self.line_gy, = self.ax2.plot([], [], 'g-', label='GY', linewidth=1)
        self.line_gz, = self.ax2.plot([], [], 'b-', label='GZ', linewidth=1)
        self.ax2.legend(loc='upper right')
        self.ax2.set_ylim(-5, 5)
        
        # Plot 3: PID Outputs
        self.ax3 = self.axes[2]
        self.ax3.set_ylabel('PID Output')
        self.ax3.set_title('PID Controller Outputs')
        self.ax3.grid(True, alpha=0.3)
        self.line_pr, = self.ax3.plot([], [], 'r-', label='Roll PID', linewidth=1)
        self.line_pp, = self.ax3.plot([], [], 'g-', label='Pitch PID', linewidth=1)
        self.line_py, = self.ax3.plot([], [], 'b-', label='Yaw PID', linewidth=1)
        self.ax3.legend(loc='upper right')
        self.ax3.set_ylim(-500, 500)
        
        # Plot 4: Motor Outputs
        self.ax4 = self.axes[3]
        self.ax4.set_xlabel('Time (seconds)')
        self.ax4.set_ylabel('PWM')
        self.ax4.set_title('Motor Outputs')
        self.ax4.grid(True, alpha=0.3)
        self.line_m1, = self.ax4.plot([], [], 'r-', label='M1 (FR)', linewidth=1)
        self.line_m2, = self.ax4.plot([], [], 'g-', label='M2 (RL)', linewidth=1)
        self.line_m3, = self.ax4.plot([], [], 'b-', label='M3 (FL)', linewidth=1)
        self.line_m4, = self.ax4.plot([], [], 'y-', label='M4 (RR)', linewidth=1)
        self.ax4.legend(loc='upper right')
        self.ax4.set_ylim(900, 2100)
        
        plt.tight_layout()
    
    def update(self, frame):
        """Update all plots with latest data"""
        
        with self.reader.lock:
            data = self.reader.data
            
            if len(data.time) == 0:
                return
        
            t = list(data.time)
            
            # Update attitude plot
            self.line_roll.set_data(t, list(data.roll))
            self.line_pitch.set_data(t, list(data.pitch))
            self.line_yaw.set_data(t, list(data.yaw))
            
            # Update gyro plot
            self.line_gx.set_data(t, list(data.gyro_x))
            self.line_gy.set_data(t, list(data.gyro_y))
            self.line_gz.set_data(t, list(data.gyro_z))
            
            # Update PID plot
            self.line_pr.set_data(t, list(data.pid_roll))
            self.line_pp.set_data(t, list(data.pid_pitch))
            self.line_py.set_data(t, list(data.pid_yaw))
            
            # Update motor plot
            self.line_m1.set_data(t, list(data.motor1))
            self.line_m2.set_data(t, list(data.motor2))
            self.line_m3.set_data(t, list(data.motor3))
            self.line_m4.set_data(t, list(data.motor4))
            
            # Adjust X limits to show recent data
            if len(t) > 1:
                self.ax1.set_xlim(max(0, t[-1] - 10), t[-1] + 0.5)
                self.ax2.set_xlim(max(0, t[-1] - 10), t[-1] + 0.5)
                self.ax3.set_xlim(max(0, t[-1] - 10), t[-1] + 0.5)
                self.ax4.set_xlim(max(0, t[-1] - 10), t[-1] + 0.5)
        
        return (self.line_roll, self.line_pitch, self.line_yaw,
                self.line_gx, self.line_gy, self.line_gz,
                self.line_pr, self.line_pp, self.line_py,
                self.line_m1, self.line_m2, self.line_m3, self.line_m4)
    
    def run(self):
        """Start the animation"""
        ani = FuncAnimation(self.fig, self.update, interval=50, blit=False, cache_frame_data=False)
        plt.show()


# ==========================================
# MAIN
# ==========================================

def main():
    print("=" * 60)
    print("  FLIGHT CONTROLLER TELEMETRY VIEWER")
    print("=" * 60)
    
    # Create serial reader
    reader = SerialReader(port='/dev/ttyUSB0')
    
    # Try to connect, fall back to simulated data
    if not reader.connect():
        print("[Viewer] Using SIMULATED data for demonstration")
    
    # Start reading
    reader.start()
    
    # Create and run viewer
    viewer = TelemetryViewer(reader)
    
    try:
        viewer.run()
    except KeyboardInterrupt:
        print("\n[Viewer] Stopped")
    finally:
        reader.stop()


if __name__ == '__main__':
    main()