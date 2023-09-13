'''This script uses a pan and tilt servo to control a gimbal with a laser pointer attached. It uses OpenCV to detect motion and moves the laser pointer to track the motion.'''

import cv2
import numpy as np
from adafruit_servokit import ServoKit
import time
from classes import IdentifiedObject

# Create an object to access the ServoKit library
gimbal = ServoKit(channels=16) # using PCA9685

pan_servo = 0 # channel 0
pan_servo_range = [0, 180]

tilt_servo = 1 # channel 1
tilt_servo_range = [90, 160]

# Create a VideoCapture object
camera_capture = cv2.VideoCapture(0) # 0 is the default camera via USB
camera_capture_width = camera_capture.get(cv2.CAP_PROP_FRAME_WIDTH)
camera_capture_height = camera_capture.get(cv2.CAP_PROP_FRAME_HEIGHT)

def set_servos_neutral(gimbal, pan_servo, tilt_servo, pan_servo_range, tilt_servo_range):
    '''Sets the servos to the neutral position.'''
    # neutral position is the middle of the range
    pan_neutral = int((pan_servo_range[1] + pan_servo_range[0]) / 2)
    tilt_neutral = int((tilt_servo_range[1] + tilt_servo_range[0]) / 2)
    gimbal.servo[pan_servo].angle = pan_neutral
    gimbal.servo[tilt_servo].angle = tilt_neutral


def move_servo_to_position(gimbal, servo, position, servo_range):
    '''Moves the servo to the specified angle.'''
    if position in range(servo_range[0], servo_range[1]):
        gimbal.servo[servo].angle = position
    else:
        print("Position out of range")
        print("Servo: ", servo)
        print("Position: ", position)
    return


def move_servo_by_degrees(gimbal, servo, degrees, servo_range):
    '''Moves the servo by the specified number of degrees.'''
    # get the current position
    current_position = int(gimbal.servo[servo].angle)
    # calculate the new position
    new_position = current_position + degrees
    if new_position in range(servo_range[0], servo_range[1]):
        gimbal.servo[servo].angle += degrees
    else:
        print("Position out of range")
        print("Servo: ", servo)
        print("New Position: ", new_position)
        print('Range: ', servo_range)
    return


def capture_single_frame(camera_capture):
    '''Captures a single frame from the camera.'''
    ret, frame = camera_capture.read()
    if frame is None:
        print("Frame is empty")
        return False
    return frame


def get_contours(old_frame, current_frame, threshold_value=40.0):
    '''Returns the contours, threshold frame, and difference frame.'''
    old_frame_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)
    current_frame_gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
    
    old_frame_blurred = cv2.GaussianBlur(old_frame_gray, (21, 21), 0)
    current_frame_blurred = cv2.GaussianBlur(current_frame_gray, (21, 21), 0)
    
    difference = cv2.absdiff(old_frame_blurred, current_frame_blurred)
    
    ret, thresh_image = cv2.threshold(difference, threshold_value, 255, cv2.THRESH_BINARY)
    thresh_image = cv2.dilate(thresh_image, None, iterations=2)

    contours, _ = cv2.findContours(thresh_image.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    return contours, thresh_image, difference


def combine_contours(contours):
    '''Returns the min and max x and y values for the contours.'''
    min_x, min_y, max_x, max_y = 10000, 10000, 0, 0     
    for contour in contours:
        if cv2.contourArea(contour) < 500:
            continue

        x, y, w, h = cv2.boundingRect(contour)
        # update min and max x and y based on contour
        min_x = min(x, min_x)
        min_y = min(y, min_y)
        max_x = max(x + w, max_x)
        max_y = max(y + h, max_y)
    return min_x, min_y, max_x, max_y


def get_largest_contour(contours, min_area=100, max_area=10000):
    '''Returns the largest contour.'''
    largest_contour = None
    largest_contour_area = 0
    for contour in contours:
        if cv2.contourArea(contour) < min_area or cv2.contourArea(contour) > max_area:
            continue

        if cv2.contourArea(contour) > largest_contour_area:
            largest_contour = contour
            largest_contour_area = cv2.contourArea(contour)
    return largest_contour


def draw_bounding_box(frame, x, y, w, h):
    '''Draws a bounding box around the identified object.'''
    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
    return frame


def track_object(identified_object, camera_capture_width, camera_capture_height):
    '''Moves the laser pointer on the gimbal to the center of the identified object.'''
    
    # get the center of the identified object
    center_x, center_y = identified_object.center

    # get the center of the camera capture
    center_camera_capture_x = int(camera_capture_width / 2)
    center_camera_capture_y = int(camera_capture_height / 2)

    # calculate the difference between the center of the identified object and the center of the camera capture
    difference_x = center_camera_capture_x - center_x
    difference_y = center_camera_capture_y - center_y

    print(f'Center of identified object: {center_x}, {center_y}')
    print(f'Center of camera capture: {center_camera_capture_x}, {center_camera_capture_y}')
    print(f'Difference: {difference_x}, {difference_y}')
    print()

    # move the pan servo
    if difference_x > 0:
        print('Moving left')
        # move left
        move_servo_by_degrees(gimbal, pan_servo, 3, pan_servo_range)
    elif difference_x < 0:
        print('Moving right')
        # move right
        move_servo_by_degrees(gimbal, pan_servo, -3, pan_servo_range)

    # move the tilt servo
    if difference_y > 0:
        print('Moving up')
        # move up
        move_servo_by_degrees(gimbal, tilt_servo, 3, tilt_servo_range)
    elif difference_y < 0:
        print('Moving down')
        # move down
        move_servo_by_degrees(gimbal, tilt_servo, -3, tilt_servo_range)

    # update the trackbar positions
    cv2.setTrackbarPos("pan", "Servo Control", int(gimbal.servo[pan_servo].angle))
    cv2.setTrackbarPos("tilt", "Servo Control", int(gimbal.servo[tilt_servo].angle))

    # wait for the servos to move
    time.sleep(0.1)


def main():
    '''This assumes that the center of the gimbal is the center of the camera capture.'''

    # set the servos to the neutral position
    set_servos_neutral(gimbal, pan_servo, tilt_servo, pan_servo_range, tilt_servo_range)
    
    # wait for the servos to move
    time.sleep(1.5)
    
    old_frame = capture_single_frame(camera_capture)
    print('Captured old frame')

    # Create a window called Servo Control
    cv2.namedWindow("Servo Control")
    cv2.createTrackbar("pan", "Servo Control", 90, 180, lambda x: None)
    cv2.createTrackbar("tilt", "Servo Control", 125, 180, lambda x: None)

    while True:

        # capture another frame
        current_frame = capture_single_frame(camera_capture)
        print('Captured current frame')

        contours, threshold_frame, difference_frame = get_contours(old_frame, current_frame)
        print(f'Found {len(contours)} contours')

        if len(contours) > 0:
            largest_countour = get_largest_contour(contours)
            
            # draw a bounding box around the largest contour
            x, y, w, h = cv2.boundingRect(largest_countour)
            current_frame = draw_bounding_box(current_frame, x, y, w, h)
            identified_object = IdentifiedObject(x, y, w, h)
            # track_object(identified_object, camera_capture_width, camera_capture_height)

        # # show the frame
        cv2.imshow("Frame with object", current_frame)
        cv2.imshow("Threshold Frame", threshold_frame)
        print()
        # cv2.imshow("Difference Frame", difference_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            # cleanup
            camera_capture.release()
            cv2.destroyAllWindows()
            break

if __name__ == "__main__":
    main()