'''This script uses a pan and tilt servo to control a gimbal with camera attached to it. Once it detects motion, it draws a box around it and uses the cv2.matchTemplate function to find the center of the object. It then moves the gimbal to the center of the object.'''

# import the necessary packages
import time
import cv2
import board
import busio

from adafruit_servokit import ServoKit
from servo_gimbal_functions import set_servos_neutral
from camera_functions import capture_single_frame, get_cropped_object_image, get_contours, get_largest_contour
from tracking_functions import track_object_servos


def main():
    '''Main function.'''
    # The FT232H must be connected to the computer via USB and env variable BLINKA_FT232H must be set to 1
    # Create an I2C device based on the FT232H object
    i2c = busio.I2C(board.SCL, board.SDA)

    # Create an object to access the ServoKit library
    gimbal = ServoKit(channels=16, i2c=i2c) # using PCA9685

    pan_servo = 0 # channel 0
    pan_servo_range = [0, 180]

    tilt_servo = 1 # channel 1
    tilt_servo_range = [90, 160]

    # Create a VideoCapture object
    camera_capture = cv2.VideoCapture(0) # 0 is the default camera via USB

    # # set the width and height
    camera_capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    camera_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # set the servos to the neutral position
    set_servos_neutral(gimbal, pan_servo, tilt_servo, pan_servo_range, tilt_servo_range)
    
    # wait for the servos to move
    time.sleep(1)
    
    background_frame = capture_single_frame(camera_capture)

    CONTOUR_THRESHOLD_VALUE = 40.0 # pixel value threshold for contour detection
    MIN_AREA = 10 # minimum area of the contour
    MAX_AREA = 500 # maximum area of the contour
    TEMPLATE_MATCHING_THRESHOLD = 0.50 # threshold for template matching
    OBJECT_BUFFER = 3 # number of pixels to add to each side of the contour when cropping the object

    print('-'*30)
    print('Sentry Camera Armed')
    print('-'*30)

    while True:

        captured_object = False

        # capture another frame
        current_frame = capture_single_frame(camera_capture)

        contours, threshold_frame, difference_frame = get_contours(background_frame, current_frame, threshold_value=CONTOUR_THRESHOLD_VALUE)

        if len(contours) > 0:
            largest_countour = get_largest_contour(contours, min_area=MIN_AREA, max_area=MAX_AREA) # find the object with the largest contour
            
            # get the x, y, width, and height of box around the contour
            x_of_contour, y_of_contour, width_of_contour, height_of_contour = cv2.boundingRect(largest_countour)

            # if the contour is too close to the edge of the frame, ignore it
            if x_of_contour < 10 or y_of_contour < 10 or x_of_contour + width_of_contour > 630 or y_of_contour + height_of_contour > 470:
                print('Contour too close to edge of frame')
                continue
            captured_object = True

        # display the frames
        # cv2.imshow("Threshold Frame", threshold_frame)
        cv2.imshow("Motion Detection", current_frame)

        if captured_object:
            # add pixels to each side of the contour to get a buffer
            x_of_contour -= OBJECT_BUFFER
            y_of_contour -= OBJECT_BUFFER
            width_of_contour += 2*OBJECT_BUFFER
            height_of_contour += 2*OBJECT_BUFFER

            cropped_object_image = get_cropped_object_image(current_frame, x_of_contour, y_of_contour, width_of_contour, height_of_contour) # crop the object from the current frame

            

            # display the cropped object image at a larger size (debugging)
            cv2.imshow("Tracking Object (original)", cv2.resize(cropped_object_image, (640, 480)))

            print('Tracking object')

            # switch to tracking object
            track_object_servos(camera_capture,
                         cropped_object_image,
                         gimbal,
                         pan_servo,
                         tilt_servo,
                         pan_servo_range,
                         tilt_servo_range,
                         template_matching_threshold=TEMPLATE_MATCHING_THRESHOLD)

            print('Finished tracking object')
            print('Moving to neutral position and capturing new background frame')
            print()
            
            set_servos_neutral(gimbal, pan_servo, tilt_servo, pan_servo_range, tilt_servo_range)
            time.sleep(1)

            # capture a new background frame
            background_frame = capture_single_frame(camera_capture)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            # cleanup
            camera_capture.release()
            cv2.destroyAllWindows()
            break

if __name__ == "__main__":
    main()