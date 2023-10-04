'''Use arrow keys to move the gimbal manually'''

# import the necessary packages
import time
import cv2
from ultralytics import YOLO
import math

from camera_functions import get_contours
from stepper_gimbal_functions import calibrate_steppers, move_gimbal_with_keypress, read_keypress
from classes import MotorDirection, MotorSpeed, MotorState, SerialConnection


def main():
    '''Main function.'''
    # initialize serial connection
    connection = SerialConnection()

    # initialize the camera
    camera_capture = cv2.VideoCapture(0)

    WIDTH = 640
    HEIGHT = 480
    
    camera_capture.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    camera_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    # camera_capture.set(cv2.CAP_PROP_FPS, 60)
    # camera_capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    USE_YOLO = False
    if USE_YOLO:
        # model
        model = YOLO("yolov8n.pt")

    CONTOUR_THRESHOLD_VALUE = 70.0 # pixel value threshold for contour detection

    # object classes
    classNames = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
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

    # wait for the Arduino to initialize
    time.sleep(3)
    print('Arduino initialized')
    print('-'*30)

    # send calibration message
    print('Calibrating...')
    calibrate_steppers(connection)
    print('Calibration complete')
    print('-'*30)

    print('Sentry Camera Armed')
    print('-'*30)
    
    # initialize motor states
    tilt_state = MotorState(MotorDirection.Zero, MotorSpeed.Off)
    pan_state = MotorState(MotorDirection.Zero, MotorSpeed.Off)

    ret, background_frame = camera_capture.read()
    if ret is False:
        print('Error reading frame')
        return

    while True:

        return_value_webcam, current_frame = camera_capture.read()
        if return_value_webcam is False:
            print('Error reading frame')
            continue

        if USE_YOLO:
            results = model(current_frame, stream=True)

            # coordinates
            for r in results:
                boxes = r.boxes

                for box in boxes:
                    # bounding box
                    x1, y1, x2, y2 = box.xyxy[0]
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2) # convert to int values

                    # put box in cam
                    cv2.rectangle(current_frame, (x1, y1), (x2, y2), (255, 0, 255), 3)

                    # confidence
                    confidence = math.ceil((box.conf[0]*100))/100
                    # add confidence to box
                    cv2.putText(current_frame, str(confidence), (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)

                    # class name
                    cls = int(box.cls[0])

                    # object details
                    org = [x1, y1]
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    fontScale = 1
                    color = (255, 0, 0)
                    thickness = 2

                    cv2.putText(current_frame, classNames[cls], org, font, fontScale, color, thickness)
        else:
            contours, threshold_frame, difference_frame = get_contours(background_frame, current_frame, threshold_value=CONTOUR_THRESHOLD_VALUE)
            for contour in contours:
                # get the x, y, width, and height of box around the contour
                x_of_contour, y_of_contour, width_of_contour, height_of_contour = cv2.boundingRect(contour)
                if x_of_contour < 10 or y_of_contour < 10:
                    continue
                if x_of_contour + width_of_contour > WIDTH or y_of_contour + height_of_contour > HEIGHT:
                    continue

                # draw a rectangle around the contour
                cv2.rectangle(current_frame, (x_of_contour, y_of_contour), (x_of_contour + width_of_contour, y_of_contour + height_of_contour), (0, 255, 0), 2)

        cv2.imshow("Webcam View", current_frame)

        # keypress = read_keypress()

        # move_gimbal_with_keypress(connection, keypress, pan_state, tilt_state)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            # cleanup the camera and close any open windows
            print('Exiting...')
            camera_capture.release()
            cv2.destroyAllWindows()
            break

        # new background frame
        # if keypress == ord('b'):
        #     ret, background_frame = camera_capture.read()
        #     print('New background frame')

if __name__ == "__main__":
    main()
