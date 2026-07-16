"""
Bridge between Python simulation and QEMU-emulated firmware
"""

import struct
import socket
import threading
import time
import numpy as np


class QEMUSerialBridge:
    """Connects simulation to QEMU via virtual serial port"""
    
    def __init__(self, socket_path='/tmp/qemu_serial.sock'):
        self.socket_path = socket_path
        self.sock = None
        self.running = False
        
        # Shared data (thread-safe)
        self.lock = threading.Lock()
        self.motor_pwm = np.array([1000, 1000, 1000, 1000], dtype=np.uint16)
        self.sensor_data = {
            'gyro': np.zeros(3),
            'accel': np.zeros(3)
        }
        
    def connect(self):
        """Connect to QEMU's virtual serial socket"""
        try:
            self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            self.sock.connect(self.socket_path)
            self.sock.settimeout(0.001)  # Non-blocking
            print(f"[Bridge] Connected to QEMU at {self.socket_path}")
            return True
        except Exception as e:
            print(f"[Bridge] Connection failed: {e}")
            return False
    
    def start(self):
        """Start communication threads"""
        self.running = True
        
        # Thread to read from QEMU (motor commands)
        read_thread = threading.Thread(target=self._read_loop)
        read_thread.daemon = True
        read_thread.start()
        
        # Thread to write to QEMU (sensor data)
        write_thread = threading.Thread(target=self._write_loop)
        write_thread.daemon = True
        write_thread.start()
    
    def stop(self):
        """Stop communication"""
        self.running = False
        if self.sock:
            self.sock.close()
    
    def set_sensor_data(self, gyro, accel):
        """Set latest sensor data to send to firmware"""
        with self.lock:
            self.sensor_data['gyro'] = np.array(gyro)
            self.sensor_data['accel'] = np.array(accel)
    
    def get_motor_commands(self):
        """Get latest motor commands from firmware"""
        with self.lock:
            return self.motor_pwm.copy()
    
    def _read_loop(self):
        """Read motor commands from firmware"""
        buffer = b''
        
        while self.running:
            try:
                data = self.sock.recv(1024)
                if data:
                    buffer += data
                    
                    # Parse complete packets (8 bytes = 4 uint16)
                    while len(buffer) >= 8:
                        packet = buffer[:8]
                        buffer = buffer[8:]
                        
                        motors = struct.unpack('<HHHH', packet)
                        with self.lock:
                            self.motor_pwm = np.array(motors, dtype=np.uint16)
                            
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"[Bridge] Read error: {e}")
                break
    
    def _write_loop(self):
        """Send sensor data to firmware"""
        while self.running:
            try:
                with self.lock:
                    gyro = self.sensor_data['gyro']
                    accel = self.sensor_data['accel']
                
                # Pack as 6 floats (24 bytes)
                packet = struct.pack('<ffffff',
                    gyro[0], gyro[1], gyro[2],
                    accel[0], accel[1], accel[2])
                
                self.sock.send(packet)
                
            except Exception as e:
                if self.running:
                    print(f"[Bridge] Write error: {e}")
                break
            
            time.sleep(0.000125)  # 8kHz rate