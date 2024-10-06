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
    time.sleep(1)
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

    user_input = input("Enter the object to track: ")
    if user_input not in class_names:
        print('Invalid object to track')
        return
    print("Confirmed tracking of:", user_input)
    
    # find the index of the object_to_track in the class_names list
    object_index_for_tracking = class_names.index(user_input)
    
    # initialize motor states
    tilt_state = MotorState(MotorDirection.Zero, MotorSpeed.Off)
    pan_state = MotorState(MotorDirection.Zero, MotorSpeed.Off)

    while True:
        ret, current_frame = camera_capture.read() # capture a frame
        if ret is False:
            print('Error reading frame')
            continue
        
        results = model(current_frame, stream=True)
        
        # get the object to track
        object_to_track = None
        for result in results.xyxy[0]:
            if result[5] == object_index_for_tracking:
                object_to_track = result
                break

        if object_to_track is None:
            print('Object not found in frame')
            set_neutral(connection) # set the steppers to neutral
            continue # go to the next frame
        else:
            # get the center of the object
            x1, y1, x2, y2 = object_to_track[:4]
            object_center = (x1 + x2) / 2, (y1 + y2) / 2

            # get the center of the frame
            frame_center = WIDTH / 2, HEIGHT / 2

            # calculate the difference between the center of the frame and the center of the match
            difference_x = object_center[0] - frame_center[0]
            difference_y = object_center[1] - frame_center[1]

            print('Difference:', difference_x, difference_y)

            center_threshold = 200 # number of pixels away from center

            # case Speed1: return 2000;
            # case Speed2: return 1500;
            # case Speed3: return 1000;
            # case Speed4: return 500;
            # case Speed5: return 375;
            # case Speed6: return 250;
            # case Speed7: return 175;

            # if object outside of deadzone, move the steppers
            if abs(difference_x) > center_threshold:
                if difference_x > 0: # left
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
                if difference_y > 0: # up
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

            # move the steppers
            move_steppers(connection, pan_state, tilt_state, serial_queue)

        if  cv2.waitKey(1) & 0xFF == ord('q'):
            # cleanup
            print('Closing Program')
            break


if __name__ == "__main__":
    main()