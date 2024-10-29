import time
import cv2
import atexit
import numpy as np
from ultralytics import YOLO
from gimbal_control_system import GimbalController

class YOLOTracker:
    def __init__(self, camera_id=0, model_path="models/yolov8n.pt"):
        # Initialize camera
        self.camera = cv2.VideoCapture(camera_id)
        self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.width = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        # Initialize YOLO model
        self.model = YOLO(model_path)
        
        # Initialize gimbal controller
        self.gimbal = GimbalController()
        
        # System state
        self.is_initialized = False
        self.target_lost_time = None
        self.return_to_neutral_delay = 3.0  # seconds to wait before returning to neutral
        
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
        
        # Visualization parameters
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.tracking_color = (255, 0, 255)  # Magenta
        
        # Register cleanup
        atexit.register(self.cleanup)
        
        # Verify camera
        assert self.camera.isOpened(), 'Camera not found'
    
    def initialize_system(self):
        """Initialize the system by homing the gimbal"""
        print("Initializing system...")
        print("Starting homing sequence...")
        
        if self.gimbal.home():
            self.is_initialized = True
            print("System initialization complete!")
            return True
        else:
            print("Failed to initialize system: Homing failed")
            return False
    
    def detect_objects(self, frame):
        """Run YOLO detection on frame"""
        results = self.model(frame, stream=True)
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
    
    def draw_tracking_info(self, frame, box, confidence, target_position):
        """Draw tracking visualization on frame"""
        # Draw system status
        status_text = "INITIALIZED" if self.is_initialized else "NOT INITIALIZED"
        cv2.putText(frame, f"System: {status_text}", 
                   (10, 30), self.font, 0.7, (0, 255, 0) if self.is_initialized else (0, 0, 255), 2)
        
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
    
    def handle_target_loss(self):
        """Handle behavior when target is lost"""
        if self.target_lost_time is None:
            self.target_lost_time = time.time()
            self.gimbal.set_velocity(0, 0)  # Stop movement
        elif time.time() - self.target_lost_time >= self.return_to_neutral_delay:
            print("Target lost - returning to neutral position")
            self.gimbal.move_to_neutral()
    
    def run(self):
        """Main tracking loop"""
        print('Starting tracking system...')
        print('-' * 30)
        
        # Initialize system
        if not self.initialize_system():
            print("Failed to initialize system. Exiting...")
            return
        
        try:
            while True:
                # Capture frame
                ret, frame = self.camera.read()
                if not ret:
                    print('Error reading frame')
                    continue
                
                # Detect objects
                boxes, confidences, class_ids = self.detect_objects(frame)
                
                # Find target object
                target_position, confidence, box = self.find_target_object(
                    boxes, confidences, class_ids)
                
                # Update tracking
                if target_position is not None:
                    # Track object using gimbal controller
                    self.gimbal.track_object(
                        target_position, (self.width, self.height))
                else:
                    # Handle target loss behavior
                    self.handle_target_loss()
                
                # Draw tracking visualization
                self.draw_tracking_info(frame, box, confidence, target_position)
                
                # Display frame
                cv2.imshow("Tracking View", frame)
                
                # Check for exit
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print('Closing Program')
                    break
                elif key == ord('h'):
                    print('Re-homing system...')
                    self.initialize_system()
                elif key == ord('n'):
                    print('Moving to neutral position...')
                    self.gimbal.move_to_neutral()
                
        except KeyboardInterrupt:
            print('Program interrupted by user')
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Cleanup resources"""
        print("Cleaning up...")
        self.camera.release()
        cv2.destroyAllWindows()
        self.gimbal.close()

def main():
    tracker = YOLOTracker()
    # tracker.run()

if __name__ == "__main__":
    main()