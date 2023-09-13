'''This script uses a pan and tilt servo to control a gimbal with camera attached to it. Once it detects motion, it draws a box around it and uses the cv2.matchTemplate function to find the center of the object. It then moves the gimbal to the center of the object.'''

import os
import time

import numpy as np
import cv2

import board
import busio

from adafruit_servokit import ServoKit
from gimbal_functions import set_servos_neutral, move_gimbal_by_difference
from contour_functions import get_contours, get_largest_contour


def capture_single_frame(camera_capture):
    '''Captures a single frame from the camera.'''
    ret, frame = camera_capture.read()
    if frame is None:
        print("Frame is empty")
        return False
    return frame


def draw_bounding_box(frame, x, y, w, h):
    '''Draws a bounding box around the identified object.'''
    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
    return frame


def check_image_match(full_image_to_search, cropped_object_image, match_method=cv2.TM_CCOEFF_NORMED):
    """
    Look for small_image in full_image and return best and worst results
    For More Info See
    http://docs.opencv.org/3.1.0/d4/dc6/tutorial_py_template_matching.html
    """
    # returns a grayscale image, where each pixel denotes how much does the neighbourhood of that pixel match with template
    result = cv2.matchTemplate(full_image_to_search, cropped_object_image, match_method)

    # Process result data and return probability values and
    # xy Location of best and worst image match
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
    return max_val, max_loc


def get_cropped_object_image(frame, x, y, w, h):
    '''Returns the cropped image.'''
    cropped_object_image = frame[y:y+h, x:x+w]
    return cropped_object_image


def track_object(camera_capture,
                 cropped_object_image,
                 camera_capture_width,
                 camera_capture_height,
                 gimbal,
                 pan_servo,
                 tilt_servo,
                 pan_servo_range,
                 tilt_servo_range,
                 servo_adjustment_speed=2):
    
    '''Matches the center of the identified object to the center of the camera capture and then uses the difference to move the servos. The next frame is then captured and the process is repeated using the cv2.matchTemplate function.'''

    while True:
        # read new frame after cropping the object
        ret, frame = camera_capture.read()

        # perform template matching
        max_val, max_loc = check_image_match(frame, cropped_object_image)
        if max_val < 0.93:
            print('No match found')
            return
        
        # # draw a bounding box around the object
        # x, y = max_loc
        # h, w, _ = cropped_object_image.shape[::-1]
        # frame = draw_bounding_box(frame, x, y, w, h)

        # # display the frame
        # cv2.imshow("Frame", frame)

        # get the difference between the center of the template match and the center of the camera capture
        difference_x = int(camera_capture_width / 2) - max_loc[0]
        difference_y = int(camera_capture_height / 2) - max_loc[1]

        # print(f'Difference x: {difference_x}')
        # print(f'Difference y: {difference_y}')

        # move the gimbal
        move_gimbal_by_difference(gimbal, pan_servo, tilt_servo, difference_x, difference_y, pan_servo_range, tilt_servo_range, servo_adjustment_speed)

        # wait
        time.sleep(.3)


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

    # set the width and height
    camera_capture_width = camera_capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    camera_capture_height = camera_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # camera_capture_width = camera_capture.get(cv2.CAP_PROP_FRAME_WIDTH)
    # camera_capture_height = camera_capture.get(cv2.CAP_PROP_FRAME_HEIGHT)

    # set the servos to the neutral position
    set_servos_neutral(gimbal, pan_servo, tilt_servo, pan_servo_range, tilt_servo_range)
    
    # wait for the servos to move
    time.sleep(1.5)
    
    background_frame = capture_single_frame(camera_capture)

    THRESHOLD_VALUE = 50.0
    MIN_AREA = 100
    MAX_AREA = 10000
    SERVO_ADDJUSTMENT_SPEED = 4

    while True:

        # capture another frame
        current_frame = capture_single_frame(camera_capture)

        contours, threshold_frame, difference_frame = get_contours(background_frame, current_frame, threshold_value=THRESHOLD_VALUE)
        # print(f'Found {len(contours)} contours')

        # resize the frame to be half the size
        current_frame = cv2.resize(current_frame, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
        threshold_frame = cv2.resize(threshold_frame, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)

        if len(contours) > 0:
            largest_countour = get_largest_contour(contours, min_area=MIN_AREA, max_area=MAX_AREA) # find the object with the largest contour
            
            x, y, w, h = cv2.boundingRect(largest_countour) # get the x, y, width, and height of the bounding box

            # add the bounding box to the current frame
            current_frame = draw_bounding_box(current_frame, x, y, w, h)

        # display the frames
        cv2.imshow("Threshold Frame", threshold_frame)
        cv2.imshow("Current Frame", current_frame)

            # if w > 0 and h > 0:
                # cropped_object_image = get_cropped_object_image(current_frame, x, y, w, h) # crop the object from the current frame
                # display the cropped object image
                # cv2.imshow("Cropped Object Image", cropped_object_image)
                # track_object(camera_capture, cropped_object_image, camera_capture_width, camera_capture_height, gimbal, pan_servo, tilt_servo, pan_servo_range, tilt_servo_range, servo_adjustment_speed=SERVO_ADDJUSTMENT_SPEED)
                # print('Finished tracking object, moving to neutral position')
                
                # set_servos_neutral(gimbal, pan_servo, tilt_servo, pan_servo_range, tilt_servo_range)
                # time.sleep(2)
                
                # go back to the beginning of the while loop and capture another frame
                # continue

            # else:
            #     print("Error: Invalid dimensions for bounding rectangle: (", x, y, w, h, ")")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            # cleanup
            camera_capture.release()
            cv2.destroyAllWindows()
            break

if __name__ == "__main__":
    main()