'''This script uses a pan and tilt servo to control a gimbal with camera attached to it. Once it detects motion, it draws a box around it and uses the cv2.matchTemplate function to find the center of the object. It then moves the gimbal to the center of the object.'''

import cv2
import numpy as np
from adafruit_servokit import ServoKit
import time
from gimbal_functions import set_servos_neutral, move_gimbal_by_difference


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


def combine_contours(contours, user_param_min_area, user_param_max_area):
    '''Returns the min and max x and y values for the contours.'''
    min_x, min_y, max_x, max_y = 10000, 10000, 0, 0   
    for contour in contours:
        if cv2.contourArea(contour) < 100 or cv2.contourArea(contour) > 1000:
            continue

        x, y, w, h = cv2.boundingRect(contour)
        
        # update min and max x and y based on contour
        min_x = min(x, min_x)
        min_y = min(y, min_y)
        max_x = max(x + w, max_x)
        max_y = max(y + h, max_y)

    return min_x, min_y, max_x, max_y


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
        if max_val < 0.7:
            print('No match found')
            return
        
        # draw a bounding box around the object
        x, y = max_loc
        h, w, _ = cropped_object_image.shape[::-1]
        frame = draw_bounding_box(frame, x, y, w, h)

        # display the frame
        cv2.imshow("Frame", frame)

        # get the difference between the center of the template match and the center of the camera capture
        difference_x = int(camera_capture_width / 2) - max_loc[0]
        difference_y = int(camera_capture_height / 2) - max_loc[1]

        print(f'Difference x: {difference_x}')
        print(f'Difference y: {difference_y}')

        # move the gimbal
        move_gimbal_by_difference(gimbal, pan_servo, tilt_servo, difference_x, difference_y, pan_servo_range, tilt_servo_range, servo_adjustment_speed)

        # wait one second
        time.sleep(2)


def main():

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

    # set the servos to the neutral position
    set_servos_neutral(gimbal, pan_servo, tilt_servo, pan_servo_range, tilt_servo_range)
    
    # wait for the servos to move
    time.sleep(1.5)
    
    original_frame = capture_single_frame(camera_capture)
    # print('Captured old frame')

    THRESHOLD_VALUE = 50.0
    MIN_AREA = 1000
    MAX_AREA = 10000
    SERVO_ADDJUSTMENT_SPEED = 4

    while True:

        # capture another frame
        current_frame = capture_single_frame(camera_capture)

        contours, threshold_frame, difference_frame = get_contours(original_frame, current_frame, threshold_value=THRESHOLD_VALUE)
        print(f'Found {len(contours)} contours')

        # display the threshold frame
        cv2.imshow("Threshold Frame", threshold_frame)

        if len(contours) > 0:
            largest_countour = get_largest_contour(contours, min_area=MIN_AREA, max_area=MAX_AREA) # find the object with the largest contour

            # combine the contours into a single bounding box
            # min_x, min_y, max_x, max_y = combine_contours(contours, user_param_min_area=MIN_AREA, user_param_max_area=MAX_AREA)

            # draw a bounding box around the combined contours
            # x, y, w, h = min_x, min_y, max_x - min_x, max_y - min_y
            
            x, y, w, h = cv2.boundingRect(largest_countour) # get the x, y, width, and height of the bounding box

            # display the bounding box in the current frame
            current_frame = draw_bounding_box(current_frame, x, y, w, h)

            # display the frame
            cv2.imshow("Frame", current_frame)

            if w > 0 and h > 0:
                cropped_object_image = get_cropped_object_image(current_frame, x, y, w, h) # crop the object from the current frame
                # display the cropped object image
                cv2.imshow("Cropped Object Image", cropped_object_image)
                # track_object(camera_capture, cropped_object_image, camera_capture_width, camera_capture_height, gimbal, pan_servo, tilt_servo, pan_servo_range, tilt_servo_range, servo_adjustment_speed=SERVO_ADDJUSTMENT_SPEED)
                # set_servos_neutral(gimbal, pan_servo, tilt_servo, pan_servo_range, tilt_servo_range)
                # time.sleep(1.5)

            else:
                print("Error: Invalid dimensions for bounding rectangle: (", x, y, w, h, ")")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            # cleanup
            camera_capture.release()
            cv2.destroyAllWindows()
            break

if __name__ == "__main__":
    main()