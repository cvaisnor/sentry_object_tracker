import serial
import time
import queue
import threading
import json
from dataclasses import dataclass
from enum import Enum, auto, IntEnum
from typing import Tuple
import numpy as np

class CommandType(IntEnum):
    VELOCITY = 0
    NEUTRAL = 1

@dataclass
class ArduinoParameters:
    max_speed: int = 4000
    max_acceleration: int = 3000
    homing_speed: int = 2000
    velocity_to_distance: int = 1000

@dataclass
class GimbalPosition:
    pan: float = 0.0
    tilt: float = 0.0
    zoom: float = 0.0 # 0-180 degrees for zoom servo

@dataclass
class GimbalRange:
    pan_range: int = 0
    tilt_range: int = 0

@dataclass
class GimbalVelocity:
    pan: float = 0.0
    tilt: float = 0.0

class ControlMode(Enum):
    POSITION = auto()
    VELOCITY = auto()

class GimbalController:
    def __init__(self, port: str = '/dev/ttyACM0', baudrate: int = 115200):
        self.serial = serial.Serial(port, baudrate, timeout=0.1)
        self.position = GimbalPosition()
        self.velocity = GimbalVelocity()
        self.control_mode = ControlMode.VELOCITY
        self.parameters = ArduinoParameters() # Arduino parameters

        # Control parameters
        self.max_velocity = 1000
        self.deadzone = 50
        self.frame_center = None
        
        # Command queue and threading
        self.command_queue = queue.Queue(maxsize=1)
        self.running = True
        self.command_thread = threading.Thread(target=self._command_worker, daemon=True)
        self.command_thread.start()

        # Rate limiting
        self.last_feedback_time = 0
        self.feedback_timeout = 1.0  # seconds
        self.min_command_interval = 0.02
        self.last_command_time = 0
        
        time.sleep(2)
    
    def process_serial_feedback(self):
        """Process position and endstop feedback from Arduino"""
        while self.serial.in_waiting:
            try:
                line = self.serial.readline().decode().strip()
                if line:
                    if line.startswith('P:'):
                        # Parse feedback (format: "P:1234,T:5678,Z:180")
                        parts = line.split(',')
                        self.position.pan = int(parts[0][2:])
                        self.position.tilt = int(parts[1][2:])
                        self.position.zoom = int(parts[2][2:])
                        self.last_feedback_time = time.time()

            except (ValueError, IndexError, UnicodeDecodeError) as e:
                print(f"Error processing feedback: {e}")
                pass

    def update_parameters(self, params: dict):
        """Update one or more parameters on the Arduino"""
        # Validate parameters
        valid_params = {
            'max_speed': (0, 4000),
            'max_acceleration': (0, 4000),
            'homing_speed': (0, 4000),
            'velocity_to_distance': (0, 2000)
        }
        
        # Filter and validate parameters
        update_params = {}
        for key, value in params.items():
            if key in valid_params:
                min_val, max_val = valid_params[key]
                if min_val <= value <= max_val:
                    update_params[key] = value
                else:
                    print(f"Parameter {key} value {value} out of range [{min_val}, {max_val}]")
        
        if not update_params:
            return
            
        # Convert parameters to JSON string
        param_str = json.dumps(update_params)
        
        # Convert string to bytes, limiting to 64 bytes for safety
        param_bytes = param_str.encode('utf-8')[:64]
        
        # Create command: [CMD_TYPE, LENGTH, PARAM_BYTES...]
        command = bytes([CommandType.SET_PARAM, len(param_bytes)]) + param_bytes
        self.command_queue.put(command)
        
        # Update local parameters
        for key, value in update_params.items():
            setattr(self.parameters, key, value)

    def _command_worker(self):
        """Worker thread for processing and sending commands"""
        while self.running:
            try:
                command = self.command_queue.get(timeout=0.1)
                
                # Rate limiting
                current_time = time.time()
                time_since_last = current_time - self.last_command_time
                if time_since_last < self.min_command_interval:
                    time.sleep(self.min_command_interval - time_since_last)
                
                bytes_written = self.serial.write(command)
                self.serial.flush()
                # print(f"Sent command: {list(command)}")  # Debug output
                
                self.process_serial_feedback()
                self.last_command_time = time.time()
                
            except queue.Empty:
                self.process_serial_feedback()
            except serial.SerialException as e:
                print(f"Serial communication error: {e}")
                self.running = False

    def move_to_neutral(self):
        """Move to starting position (0,0)"""
        command = bytes([CommandType.NEUTRAL, 0, 0, 0])
        self.command_queue.put(command)

    def set_velocity(self, pan_velocity: float, tilt_velocity: float, zoom_angle: float = None):
        """Set velocity for velocity control mode and zoom angle"""
        
        # Clip velocities to max range
        pan_velocity = np.clip(pan_velocity, -self.max_velocity, self.max_velocity)
        tilt_velocity = np.clip(tilt_velocity, -self.max_velocity, self.max_velocity)
        
        # Scale to byte range (0-255)
        pan_byte = int(((pan_velocity / self.max_velocity) * 127) + 128)
        tilt_byte = int(((tilt_velocity / self.max_velocity) * 127) + 128)
        
        # Only update zoom if a new angle is provided
        if zoom_angle is not None:
            zoom_angle = np.clip(zoom_angle, 0, 180)
            self.position.zoom = zoom_angle
        
        zoom_byte = int((self.position.zoom / 180) * 255)

        # Debug output
        print(f"Sending command - Raw velocities: pan={pan_velocity}, tilt={tilt_velocity}, zoom={self.position.zoom}")
        
        # Create and send command
        command = bytes([CommandType.VELOCITY, pan_byte, tilt_byte, zoom_byte])
        self.command_queue.put(command)
        
        # Store current velocities
        self.velocity.pan = pan_velocity
        self.velocity.tilt = tilt_velocity

    def track_object(self, object_position: Tuple[float, float], frame_size: Tuple[int, int]):
        """Track object based on its position in frame"""
        if self.is_homing:
            return

        if self.frame_center is None:
            self.frame_center = (frame_size[0] / 2, frame_size[1] / 2)

        # Calculate position error in pixels
        error_x = object_position[0] - self.frame_center[0]
        error_y = object_position[1] - self.frame_center[1]

        # Convert pixel error to velocity (with deadzone)
        if abs(error_x) > self.deadzone:
            pan_velocity = (error_x / frame_size[0]) * self.max_velocity
        else:
            pan_velocity = 0

        if abs(error_y) > self.deadzone:
            tilt_velocity = (error_y / frame_size[1]) * self.max_velocity
        else:
            tilt_velocity = 0

        # Pass None for zoom_angle to maintain current zoom during tracking
        self.set_velocity(pan_velocity, tilt_velocity, None)

    def close(self):
        """Cleanup resources"""
        self.running = False
        self.command_thread.join()
        self.serial.close()

# Test the GimbalController class
if __name__ == "__main__":
    try:
        gimbal = GimbalController()

        print("Issuing velocity commands...")
        gimbal.set_velocity(-1000, -400, 20) 
        time.sleep(1.5)
        gimbal.set_velocity(900, 600, 100) 
        time.sleep(1.5)

        # return to neutral position
        print("Returning to neutral position")
        gimbal.move_to_neutral()

        # time.sleep(3)
        print("Exiting...")
    
    except serial.SerialException as e:
        print(f"Failed to connect to gimbal: {e}")
    except KeyboardInterrupt:
        print("\nExiting...")
        if 'gimbal' in locals():
            gimbal.close()