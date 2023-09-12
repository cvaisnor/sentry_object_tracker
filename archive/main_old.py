'''This script uses a pan and tilt servo to control a gimbal with a webcam attached. It uses OpenCV to detect motion and moves th servos to track the motion.'''

import cv2
import numpy as np
from adafruit_servokit import ServoKit
import time
from datetime import datetime

# Create an object to access the ServoKit library
gimbal = ServoKit(channels=16) # using PCA9685

pan_servo = 0 # channel 0, can go 90 degrees either direction
pan_servo_range = [0, 180]

tilt_servo = 1 # channel 1
tilt_servo_range = [90, 160]

# Create a VideoCapture object
camera_capture = cv2.VideoCapture(0) # 0 is the default camera via USB
camera_capture_width = camera_capture.get(cv2.CAP_PROP_FRAME_WIDTH)
camera_capture_height = camera_capture.get(cv2.CAP_PROP_FRAME_HEIGHT)

# Setting Up Motion Detection
# Assigning our static_back to None
static_background = None

last_motion_time = 0

# defining functions
def set_servo_neutral(gimbal, pan_servo, tilt_servo, pan_servo_range, tilt_servo_range):
    # neutral position is the middle of the range
    pan_neutral = int((pan_servo_range[1] + pan_servo_range[0]) / 2)
    tilt_neutral = int((tilt_servo_range[1] + tilt_servo_range[0]) / 2)
    gimbal.servo[pan_servo].angle = pan_neutral
    gimbal.servo[tilt_servo].angle = tilt_neutral


def move_servo_to_position(gimbal, servo, position, servo_range):
    if position in range(servo_range[0], servo_range[1]):
        gimbal.servo[servo].angle = position
    else:
        print("Position out of range")
        print("Servo: ", servo)
        print("Position: ", position)
    return


def main():
    pass

# Set the servos to neutral
set_servo_neutral(gimbal, pan_servo, tilt_servo, pan_servo_range, tilt_servo_range)

# Create a window called Servo Control
cv2.namedWindow("Servo Control")
cv2.createTrackbar("pan", "Servo Control", 90, 180, lambda x: None)
cv2.createTrackbar("tilt", "Servo Control", 125, 180, lambda x: None)

# loop until the user presses the q key
while True:

    # Read the current frame from the webcam
    ret, frame = camera_capture.read()
    
    # check if frame is empty
    if frame is None:
        print("Frame is empty")
        break

    # Converting color image to gray_scale image
    static_background_in_loop = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Converting gray scale image to GaussianBlur
    static_background_blurred = cv2.GaussianBlur(static_background_in_loop, (21, 21), 0)

    # In first iteration we assign the value of static_back to our first frame
    if static_background is None:
        static_background = static_background_blurred
        continue

    # Difference between static background and current frame(which is GaussianBlur)
    difference_frame = cv2.absdiff(static_background, static_background_blurred)

    # If change in between static background and current frame is greater than 30 it will show white color(255)
    threshold_frame = cv2.threshold(difference_frame, 30, 255, cv2.THRESH_BINARY)[1]
    threshold_frame = cv2.dilate(threshold_frame, None, iterations = 2)

    # Finding contour of moving object
    contours, _ = cv2.findContours(threshold_frame.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        if cv2.contourArea(contour) < 500:
            continue
    
        (x, y, w, h) = cv2.boundingRect(contour)
        
        # making green rectangle around the moving object
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)

        # Calculate the center of the contour
        center_x = int(x + w / 2)
        center_y = int(y + h / 2)

        # Draw a circle at the center of the contour
        cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
    
    # Displaying image in gray_scale
    # cv2.imshow("Original Frame", static_background)    
    
    # Displaying the difference in currentframe to
    # the staticframe(very first_frame)
    # cv2.imshow("Difference Frame", difference_frame)    
    
    # Displaying the black and white image in which if
    # intensity difference greater than 30 it will appear white
    # cv2.imshow("Threshold Frame", threshold_frame)
    
    # Displaying color frame with contour of motion of object
    cv2.imshow("Color Frame", frame)

    # Read the trackbar values
    pan_bar = cv2.getTrackbarPos("pan", "Servo Control")
    tilt_bar = cv2.getTrackbarPos("tilt", "Servo Control")

    # Move the servos to the trackbar positions
    move_servo_to_position(gimbal, pan_servo, pan_bar, pan_servo_range)
    move_servo_to_position(gimbal, tilt_servo, tilt_bar, tilt_servo_range)

    # every second, update the static background
    if time.time() - last_motion_time > 0.3:
        static_background = static_background_blurred
        last_motion_time = time.time()

    # Wait for the user to press the q key
    c = cv2.waitKey(1)
    if c == ord('q'):
        break

# Release the VideoCapture object
camera_capture.release()
# Destroy all windows
cv2.destroyAllWindows()