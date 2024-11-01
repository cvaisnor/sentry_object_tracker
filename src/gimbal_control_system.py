import serial
import time
import queue
import threading
from dataclasses import dataclass
from enum import Enum, auto, IntEnum
from typing import Optional, Tuple, Union
import numpy as np

class CommandType(IntEnum):
    VELOCITY = 0
    HOME = 1
    NEUTRAL = 2

@dataclass
class GimbalPosition:
    pan: float = 0.0
    tilt: float = 0.0

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
        self.target_position = GimbalPosition()
        self.velocity = GimbalVelocity()
        self.control_mode = ControlMode.VELOCITY
        
        # System state
        self.is_homed = False
        self.is_homing = False
        self.home_timeout = 45  # seconds
        self.last_feedback_time = 0
        self.feedback_timeout = 1.0  # seconds
        
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
        self.min_command_interval = 0.02
        self.last_command_time = 0
        
        # Clear serial buffer
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()
        time.sleep(2)
        print("Gimbal controller initialized")
    
    def run_homing(self) -> bool:
        """Start homing sequence and wait for completion"""
        print("Starting homing sequence...")
        self.is_homing = True
        self.is_homed = False
        
        # Send home command
        command = bytes([CommandType.HOME, 0, 0])
        self.command_queue.put(command)
        
        # Wait for homing to complete
        start_time = time.time()
        self.last_feedback_time = time.time()
        
        while self.is_homing:
            if time.time() - start_time > self.home_timeout:
                print("Homing timed out!")
                self.is_homing = False
                return False
            
            # Check if we're still receiving feedback
            if time.time() - self.last_feedback_time > self.feedback_timeout:
                print("Lost communication with Arduino!")
                self.is_homing = False
                return False
            
            time.sleep(0.1)
        
        if not self.is_homed:
            print("Homing failed!")
            return False
        
        print("Homing completed successfully")
        return True
    
    def process_serial_feedback(self):
        """Process position feedback from Arduino"""
        while self.serial.in_waiting:
            try:
                line = self.serial.readline().decode().strip()
                if line:  # Only process non-empty lines
                    print(f"Received feedback: {line}")  # Debug output
                    if line.startswith('P:'):
                        # Parse feedback (format: "P:1234,T:5678,H:1")
                        parts = line.split(',')
                        self.position.pan = int(parts[0][2:])
                        self.position.tilt = int(parts[1][2:])
                        self.is_homed = parts[2][2:] == '1'
                        self.last_feedback_time = time.time()
                        
                        if self.is_homed and self.is_homing:
                            # print("Home state achieved")
                            self.is_homing = False
            except (ValueError, IndexError, UnicodeDecodeError) as e:
                print(f"Error processing feedback: {e}")
                pass
    
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
                
                # Send command and verify
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
        """Move to neutral position if homed"""
        if not self.is_homed:
            print("Cannot move to neutral position: Gimbal not homed")
            return

        command = bytes([CommandType.NEUTRAL, 0, 0])
        self.command_queue.put(command)

    def set_velocity(self, pan_velocity: float, tilt_velocity: float):
        """Set velocity for velocity control mode"""
        if self.is_homing:
            return

        # Clip velocities to max range
        pan_velocity = np.clip(pan_velocity, -self.max_velocity, self.max_velocity)
        tilt_velocity = np.clip(tilt_velocity, -self.max_velocity, self.max_velocity)
        
        # Scale to byte range (0-255)
        pan_byte = int(((pan_velocity / self.max_velocity) * 127) + 128)
        tilt_byte = int(((tilt_velocity / self.max_velocity) * 127) + 128)
        
        # Create command bytes
        command = bytes([CommandType.VELOCITY, pan_byte, tilt_byte])
        
        # Debug output
        print(f"Sending command - Raw velocities: pan={pan_velocity}, tilt={tilt_velocity}")
        print(f"Converted to bytes: pan={pan_byte}, tilt={tilt_byte}")
        print(f"Command bytes: {list(command)}")
        
        # Send command
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

        self.set_velocity(pan_velocity, tilt_velocity)

    def close(self):
        """Cleanup resources"""
        self.running = False
        self.command_thread.join()
        self.serial.close()

# Test the GimbalController class
if __name__ == "__main__":
    try:
        gimbal = GimbalController()
        success = gimbal.run_homing()
        if not success:
            print("Homing failed")
    
        # issue a velocity commands
        gimbal.set_velocity(-1000, -400) 
        time.sleep(1.5)
        gimbal.set_velocity(900, 600) 
        time.sleep(1.5)

        # return to neutral position
        print("Returning to neutral position")
        gimbal.move_to_neutral()

        time.sleep(3)
        print("Exiting...")
    
    except serial.SerialException as e:
        print(f"Failed to connect to gimbal: {e}")
    except KeyboardInterrupt:
        print("\nExiting...")
        if 'gimbal' in locals():
            gimbal.close()