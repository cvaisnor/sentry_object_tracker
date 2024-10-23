# import the necessary packages
import time
import cv2
import atexit

from ultralytics import YOLO
from multiprocessing import Process
from camera_functions import detect_objects_yolo
from tracking_functions import track_object_yolo
from stepper_gimbal_functions import calibrate_steppers
from classes import SerialConnection, SerialMessagesQueue, MotorDirection, MotorSpeed, MotorState

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
    
    model = YOLO("models/yolov8n.pt")
    
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
        boxes, confidences, class_ids = detect_objects_yolo(current_frame, model)

        # object to track
        object_to_track = 'cell phone'
        object_coordinates = None
        object_confidence = None
        for i, cls in enumerate(class_ids):
            if class_names[cls] == object_to_track:
                object_coordinates = [(boxes[i][0] + boxes[i][2])/2, (boxes[i][1] + boxes[i][3])/2]
                object_confidence = confidences[i]
                break

        if object_coordinates is None:
            print('Object not found')
            # set_neutral(connection)
        else: # track object
            track_object_yolo(connection, pan_state, tilt_state, object_coordinates, object_confidence, WIDTH, HEIGHT, serial_queue, current_frame)
        
        cv2.imshow("Webcam View", current_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            # cleanup
            print('Closing Program')
            break


if __name__ == "__main__":
    main()