# import the necessary packages
import time
import cv2
import atexit
import math

from ultralytics import YOLO
from multiprocessing import Process
from stepper_gimbal_functions import move_steppers
from classes import SerialConnection, SerialMessagesQueue, MotorDirection, MotorSpeed, MotorState
from stepper_gimbal_functions import calibrate_steppers, set_neutral

def main():
    '''Main function.'''
    
    def close_program():
        camera_capture.release()
        cv2.destroyAllWindows()
        serial_queue_process.terminate()

    # initialize the camera
    camera_capture = cv2.VideoCapture(0)
    camera_capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG')) # codec
    WIDTH = camera_capture.get(cv2.CAP_PROP_FRAME_WIDTH)
    HEIGHT = camera_capture.get(cv2.CAP_PROP_FRAME_HEIGHT)
    
    model = YOLO("yolov8n.pt")
    
    # object classes
    class_names = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
              "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
              "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
              "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
              "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
              "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli",
              "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed",
              "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
              "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
              "teddy bear", "hair drier", "toothbrush"
              ]

    # initialize the serial connection
    connection = SerialConnection() # set port in this class
    print('Serial connection established')
    time.sleep(2)
    print('Calibrating...')
    calibrate_steppers(connection)
    print('Calibration complete')
    serial_queue = SerialMessagesQueue(connection)
    serial_queue_process = Process(target=serial_queue.start, args=())
    serial_queue_process.start()
    print('Serial queue process started')
    print('-'*30)

    assert camera_capture.isOpened(), 'Camera not found'

    atexit.register(close_program)

    print('Sentry Camera Armed')
    print('-'*30)
    
    # initialize motor states
    tilt_state = MotorState(MotorDirection.Zero, MotorSpeed.Off)
    pan_state = MotorState(MotorDirection.Zero, MotorSpeed.Off)

    while True:
        ret, current_frame = camera_capture.read() # capture a frame
        if ret is False:
            print('Error reading frame')
            continue
        
        # detect objects
        boxes, confidences, class_ids = detect_objects(current_frame, model)

        # object to track
        object_to_track = 'cell phone'
        object_coordinates = None
        object_confidence = None
        for i, cls in enumerate(class_ids):
            if class_names[cls] == object_to_track:
                object_coordinates = [(boxes[i][0] + boxes[i][2])/2, (boxes[i][1] + boxes[i][3])/2]
                object_confidence = confidences[i]
                break

        # track object
        if object_coordinates is None:
            print('Object not found')
            # set_neutral(connection)
        else:
            track_object(connection, pan_state, tilt_state, object_coordinates, object_confidence, WIDTH, HEIGHT, serial_queue, current_frame)
        
        cv2.imshow("Webcam View", current_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            # cleanup
            print('Closing Program')
            break

# function to detect objects in a frame
def detect_objects(frame, model):
    '''
    Input: frame 
    Output: three lists - boxes, confidences, class_ids
    '''
    results = model(frame, stream=True)
    boxes = []
    confidences = []
    class_ids = []

    for r in results:
        for box in r.boxes:
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            boxes.append([x1, y1, x2, y2])
            confidence = math.ceil((box.conf[0]*100))/100
            confidences.append(confidence)
            cls = int(box.cls[0])
            class_ids.append(cls)
    
    return boxes, confidences, class_ids

# function to track a detected object with the gimbal
def track_object(connection, pan_state, tilt_state, object_coordinates, confidences, width, height, serial_queue, current_frame):
    '''
    Input: serial_connection, object_coordinates, width, height
    Output: None
    '''

    # draw a rectangle around the object with the confidence
    x1, y1 = object_coordinates
    x1, y1 = int(x1), int(y1)
    cv2.rectangle(current_frame, (x1, y1), (x1+10, y1+10), (255, 0, 255), 3)
    cv2.putText(current_frame, str(confidences), (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)

    # deadzone
    center_threshold = 100

    # calculate the difference between the center of the frame and the center of the matched object
    difference_x = object_coordinates[0] - width/2
    difference_y = object_coordinates[1] - height/2

    print(f'difference_x: {difference_x}, difference_y: {difference_y}')

    # # if object outside of deadzone, move the steppers
    if abs(difference_x) > center_threshold:
        if difference_x < 0: # left
            # print('Moving left')
            pan_state.speed = MotorSpeed.Speed5
            pan_state.direction = MotorDirection.Left
            tilt_state.speed = MotorSpeed.Off
            tilt_state.direction = MotorDirection.Zero
        else: # right
            # print('Moving right')
            pan_state.speed = MotorSpeed.Speed5
            pan_state.direction = MotorDirection.Right
            tilt_state.speed = MotorSpeed.Off
            tilt_state.direction = MotorDirection.Zero

    if abs(difference_y) > center_threshold:
        if difference_y < 0: # up
            # print('Moving up')
            pan_state.speed = MotorSpeed.Off
            pan_state.direction = MotorDirection.Zero
            tilt_state.speed = MotorSpeed.Speed6
            tilt_state.direction = MotorDirection.Up
        else: # down
            # print('Moving down')
            pan_state.speed = MotorSpeed.Off
            pan_state.direction = MotorDirection.Zero
            tilt_state.speed = MotorSpeed.Speed6
            tilt_state.direction = MotorDirection.Down

    # if object inside of deadzone, stop the steppers
    if abs(difference_x) < center_threshold: # pan
        # print('no pan')
        pan_state.speed = MotorSpeed.Off

    if abs(difference_y) < center_threshold: # tilt
        # print('no tilt')
        tilt_state.speed = MotorSpeed.Off

    # # move the steppers
    move_steppers(connection, pan_state, tilt_state, serial_queue)


if __name__ == "__main__":
    main()