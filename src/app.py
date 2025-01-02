# app.py
from flask import Flask, render_template, Response, request, jsonify
import cv2
import time
from threading import Lock
import json
from ultralytics import YOLO
import numpy as np
from gimbal_control_system import GimbalController

app = Flask(__name__)

class FlaskYOLOTracker:
    def __init__(self, camera_id=0, model_path="models/yolov8n.pt"):
        # Default dimensions in case camera init fails
        self.width = 1920
        self.height = 1080  # Default height
        self.camera = None
        
        try:
            # Initialize camera
            self.camera = cv2.VideoCapture(camera_id)
            if self.camera.isOpened():
                self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
                self.width = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
                self.height = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
                print(f"Camera initialized successfully: {self.width}x{self.height}")
        except Exception as e:
            print(f"Error initializing camera: {str(e)} - running in camera-less mode")
            self.camera = None
        
        # Initialize other components
        try:
            self.current_zoom = 0
            self.last_zoom_update = time.time()
            self.zoom_update_interval = 0.1
            self.lock = Lock()
            self.model = YOLO(model_path)
            self.gimbal = GimbalController()
            
            # System state
            self.is_initialized = False
            self.target_lost_time = None
            self.return_to_neutral_delay = 3.0
            self.is_tracking_enabled = False
            self.target_class = 'cell phone'
            
            # Manual control state
            self.last_manual_command_time = time.time()
            self.manual_timeout = 0.1
            
            # Font and color settings
            self.font = cv2.FONT_HERSHEY_SIMPLEX
            self.tracking_color = (255, 0, 255)
            
            # YOLO class names
            self.class_names = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
                            "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
                            "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
                            "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
                            "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
                            "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli",
                            "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed",
                            "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
                            "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
                            "teddy bear", "hair drier", "toothbrush"]
        except Exception as e:
            print(f"Error initializing tracker components: {str(e)}")
            raise  # Re-raise the exception if critical components fail
    
    def update_zoom(self, zoom_angle):
        """Utility method to update zoom without changing velocity"""
        with self.lock:
            self.current_zoom = zoom_angle
            self.gimbal.set_velocity(
                self.gimbal.velocity.pan,
                self.gimbal.velocity.tilt,
                zoom_angle
            )

    def detect_objects(self, frame):
        """Run YOLO detection on frame"""
        results = self.model(frame, stream=True, verbose=False)
        boxes = []
        confidences = []
        class_ids = []
        
        for r in results:
            boxes.extend(r.boxes.xyxy.cpu().numpy())
            confidences.extend(r.boxes.conf.cpu().numpy())
            class_ids.extend(r.boxes.cls.cpu().numpy().astype(int))
            
        return boxes, confidences, class_ids
    
    def find_target_object(self, boxes, confidences, class_ids, target_class='cell phone'):
        """Find specific object in YOLO detections"""
        for i, cls in enumerate(class_ids):
            if self.class_names[cls] == target_class:
                box = boxes[i]
                center_x = (box[0] + box[2]) / 2
                center_y = (box[1] + box[3]) / 2
                return (center_x, center_y), confidences[i], box
        return None, None, None
    
    def draw_tracking_info(self, frame, box, confidence, target_position, target_class):
        """Draw tracking visualization on frame"""
        # Add current target class to frame
        cv2.putText(frame, 
                    f"Target: {target_class}", 
                    (10, 90), self.font, 0.7, (0, 255, 0), 2)
        
        if box is not None:
            # Draw bounding box
            cv2.rectangle(frame, 
                         (int(box[0]), int(box[1])), 
                         (int(box[2]), int(box[3])), 
                         self.tracking_color, 2)
            
            # Draw center point
            cv2.circle(frame, 
                      (int(target_position[0]), int(target_position[1])), 
                      5, self.tracking_color, -1)
            
            # Draw confidence
            cv2.putText(frame, 
                       f"Conf: {confidence:.2f}", 
                       (int(box[0]), int(box[1] - 10)), 
                       self.font, 0.5, self.tracking_color, 2)
            
            # Reset target lost time
            self.target_lost_time = None
        else:
            # Draw time until neutral position
            if self.target_lost_time is not None:
                time_lost = time.time() - self.target_lost_time
                if time_lost < self.return_to_neutral_delay:
                    remaining = self.return_to_neutral_delay - time_lost
                    cv2.putText(frame, 
                              f"Returning to neutral in: {remaining:.1f}s", 
                              (10, 60), self.font, 0.7, (0, 165, 255), 2)
        
        # Draw frame center
        cv2.circle(frame, 
                  (self.width // 2, self.height // 2), 
                  5, (0, 255, 0), -1)
        
        # Draw deadzone
        deadzone = self.gimbal.deadzone
        cv2.rectangle(frame,
                     (self.width // 2 - deadzone, self.height // 2 - deadzone),
                     (self.width // 2 + deadzone, self.height // 2 + deadzone),
                     (0, 255, 0), 1)
    
    # def handle_target_loss(self):
    #     """Handle behavior when target is lost"""
    #     if self.target_lost_time is None:
    #         self.target_lost_time = time.time()
    #         # self.gimbal.set_velocity(0, 0)  # Stop movement
    #     # elif time.time() - self.target_lost_time >= self.return_to_neutral_delay:
    #         # print("Target lost - returning to neutral position")
    #         # self.gimbal.move_to_neutral() # not blocking
    
    def get_frame(self):
        """Get processed frame with tracking visualization"""
        try:
            ret, frame = self.camera.read()
            if not ret:
                # Create a blank frame if camera read fails
                frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
                cv2.putText(frame, 
                        "No Camera Feed Available", 
                        (self.width//4, self.height//2),
                        self.font, 1, (0, 165, 255), 2)

            if self.is_tracking_enabled:
                try:
                    boxes, confidences, class_ids = self.detect_objects(frame)
                    target_position, confidence, box = self.find_target_object(
                        boxes, confidences, class_ids, target_class=self.target_class)
                    
                    if target_position is not None:
                        with self.lock:
                            self.gimbal.track_object(target_position, (self.width, self.height))
                        
                    self.draw_tracking_info(frame, box, confidence, target_position, self.target_class)
                except Exception as e:
                    print(f"Error in tracking: {str(e)}")
                    # Disable tracking if there's an error
                    self.is_tracking_enabled = False
            else:
                # Manual control logic
                current_time = time.time()
                
                # Handle movement timeout
                if current_time - self.last_manual_command_time > self.manual_timeout:
                    with self.lock:
                        self.gimbal.set_velocity(0, 0, self.current_zoom)
                
                # Update zoom periodically even when not moving
                if current_time - self.last_zoom_update > self.zoom_update_interval:
                    self.update_zoom(self.current_zoom)
                    self.last_zoom_update = current_time
            
            # Draw system status
            status_text = "TRACKING MODE" if self.is_tracking_enabled else "MANUAL MODE"
            cv2.putText(frame, f"System: {status_text}", 
                    (10, 30), self.font, 0.7, (0, 255, 0) if self.is_initialized else (0, 0, 255), 2)
            
            # Encode frame for streaming
            ret, buffer = cv2.imencode('.jpg', frame)
            if not ret:
                raise Exception("Failed to encode frame")
            return buffer.tobytes()

        except Exception as e:
            print(f"Error in get_frame: {str(e)}")
            # Return a minimal error frame if everything fails
            error_frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(error_frame, 
                    "System Error", 
                    (220, 240),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            ret, buffer = cv2.imencode('.jpg', error_frame)
            return buffer.tobytes()

        finally:
            # Ensure manual control and zoom updates continue even if video fails
            if not self.is_tracking_enabled:
                current_time = time.time()
                if current_time - self.last_manual_command_time > self.manual_timeout:
                    with self.lock:
                        self.gimbal.set_velocity(0, 0, self.current_zoom)

    def handle_manual_control(self, x_velocity, y_velocity, zoom_angle=None):
        """Handle manual control inputs with integrated zoom"""
        with self.lock:
            if zoom_angle is not None:
                self.current_zoom = zoom_angle
            self.gimbal.set_velocity(x_velocity, y_velocity, self.current_zoom)
            self.last_manual_command_time = time.time()

# Initialize tracker
tracker = FlaskYOLOTracker()

@app.route('/')
def index():
    """Serve the main page"""
    return render_template('index.html', class_names=sorted(tracker.class_names, key=str.lower)) # sorting

def generate_frames():
    """Generator function for video streaming"""
    while True:
        frame = tracker.get_frame()
        if frame is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.01)  # Small delay to prevent overwhelming the browser

@app.route('/video_feed')
def video_feed():
    """Video streaming route"""
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/control/<action>')
def control(action):
    """Handle control actions"""
    if action == 'home':
        tracker.initialize_system()
        return jsonify({'status': 'success', 'message': 'Homing system'})
    elif action == 'neutral':
        tracker.gimbal.move_to_neutral()
        return jsonify({'status': 'success', 'message': 'Moving to neutral position'})
    elif action == 'toggle_tracking':
        try:
            tracker.is_tracking_enabled = not tracker.is_tracking_enabled
            status = 'enabled' if tracker.is_tracking_enabled else 'disabled'
            return jsonify({'status': 'success', 'message': f'Tracking {status}'})
        except Exception as e:
            print(f"Error toggling tracking: {str(e)}")  # Add error logging
            return jsonify({'status': 'error', 'message': str(e)})
    return jsonify({'status': 'error', 'message': 'Invalid action'})

@app.route('/manual_control', methods=['POST'])
def manual_control():
    """Handle manual control commands from joystick with integrated zoom"""
    try:
        data = request.get_json()
        x_velocity = float(data['x'])  # -1 to 1
        y_velocity = float(data['y'])  # -1 to 1
        zoom_angle = data.get('zoom')  # Optional zoom angle

        # Only process manual control when tracking is disabled
        if not tracker.is_tracking_enabled:
            # Convert joystick values to velocities and include current zoom
            tracker.handle_manual_control(x_velocity, y_velocity, zoom_angle)
            return json.dumps({'status': 'success'})
        return json.dumps({'status': 'error', 'message': 'Tracking is enabled'})
    except Exception as e:
        return json.dumps({'status': 'error', 'message': str(e)})

@app.route('/set_target_class', methods=['POST'])
def set_target_class():
    """Set the target class for tracking"""
    data = request.get_json()
    new_class = data.get('class')
    if new_class in tracker.class_names:
        tracker.target_class = new_class
        return jsonify({
            'status': 'success',
            'message': f'Now tracking: {new_class}'
        })
    return jsonify({
        'status': 'error',
        'message': 'Invalid class name'
    })

@app.route('/control/get_state')
def get_state():
    """Return current tracking state"""
    return jsonify({
        'status': 'success',
        'is_tracking': tracker.is_tracking_enabled
    })

@app.route('/gimbal_position')
def gimbal_position():
    """Return current gimbal position"""
    return jsonify({
        'pan': tracker.gimbal.position.pan,
        'tilt': tracker.gimbal.position.tilt
    })

@app.route('/update_zoom', methods=['POST'])
def update_zoom():
    """Update zoom value without changing movement"""
    try:
        data = request.get_json()
        zoom_angle = float(data['angle'])
        
        if not tracker.is_tracking_enabled:  # Only update zoom in manual mode
            tracker.current_zoom = zoom_angle  # Update stored zoom value
            tracker.update_zoom(zoom_angle)   # Send zoom command
            
        return jsonify({
            'status': 'success',
            'message': f'Zoom updated to {zoom_angle}°'
        })
    except Exception as e:
        return jsonify({
            'status': 'error',
            'message': str(e)
        })

@app.route('/get_zoom')
def get_zoom():
    """Get current zoom angle"""
    return jsonify({
        'status': 'success',
        'zoom': tracker.gimbal.position.zoom
    })

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, threaded=True)