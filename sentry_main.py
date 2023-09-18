'''This script uses a pan and tilt servo to control a gimbal with camera attached to it. Once it detects motion, it draws a box around it and uses the cv2.matchTemplate function to find the center of the object. It then moves the gimbal to the center of the object.'''

# import the necessary packages
import time
import cv2
import board
import busio
import numpy as np

from adafruit_servokit import ServoKit
from gimbal_functions import set_servos_neutral, move_pan_servo, move_tilt_servo
from contour_functions import get_contours, get_largest_contour


def capture_single_frame(camera_capture):
    '''Captures a single frame from the camera.'''
    ret, frame = camera_capture.read()
    if frame is None:
        print("Frame is empty")
        return False
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
                 gimbal,
                 pan_servo,
                 tilt_servo,
                 pan_servo_range,
                 tilt_servo_range,
                 template_matching_threshold=0.70):
    
    '''Matches the center of the identified object to the center of the camera capture and then uses the difference to move the servos. The next frame is then captured and the process is repeated using the cv2.matchTemplate function.'''

    # create a video writer object for recording the video
    # fourcc = cv2.VideoWriter_fourcc(*'XVID')
    # video_writer = cv2.VideoWriter(filename='output.avi', fourcc=fourcc, fps=24.0, frameSize=(640, 480))

    # Use a list to store the last # frames
    all_cropped_objects = [cropped_object_image]

    while True:
        # read new frame after cropping the object
        frame = capture_single_frame(camera_capture)

        # perform template matching
        max_val, max_loc = check_image_match(frame, cropped_object_image)
        print('max_val: ', max_val)

        # condition to check object is matched or not
        if max_val > template_matching_threshold:
            crop_object = get_cropped_object_image(frame, max_loc[0], max_loc[1], cropped_object_image.shape[1], cropped_object_image.shape[0])
            all_cropped_objects.append(crop_object)

            # remove the oldest frame if more than 5 objects are stored
            if len(all_cropped_objects) > 2:
                all_cropped_objects.pop(1)

            # take the average of all object images in the list
            cropped_object_image = np.average(all_cropped_objects, axis=0).astype(np.uint8)
        else:
            print('Object not found, switching to motion detection')
            # video_writer.release()
            cv2.destroyAllWindows()
            return False
            
        # draw a rectangle around the match on a copy of the current frame
        frame_copy = frame.copy()
        cv2.rectangle(frame_copy, max_loc, (max_loc[0] + cropped_object_image.shape[1], max_loc[1] + cropped_object_image.shape[0]), (0, 255, 0), 2)

        # display the frame
        cv2.imshow("Now tracking", frame_copy)

        # display the new cropped object image (debugging)
        cv2.imshow("Running Average", cv2.resize(cropped_object_image, (640, 480)))

        # calculate the difference between the center of the frame and the center of the match
        difference_x = (frame.shape[1] / 2) - (max_loc[0] + cropped_object_image.shape[1] / 2)
        difference_y = (frame.shape[0] / 2) - (max_loc[1] + cropped_object_image.shape[0] / 2)

        # the larger the difference, the faster the servos move
        servo_adjustment_speed = int(abs(difference_x) / 14) + 1
        if servo_adjustment_speed > 10:
            servo_adjustment_speed = 10 # limit the speed

        # if object outside of deadzone, move the servos
        if abs(difference_x) > 10:
            move_pan_servo(gimbal, pan_servo, pan_servo_range, servo_adjustment_speed, difference_x)
        
        if abs(difference_y) > 10:
            move_tilt_servo(gimbal, tilt_servo, tilt_servo_range, servo_adjustment_speed, difference_y)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            # cleanup
            camera_capture.release()
            # video_writer.release()
            cv2.destroyAllWindows()
            break



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
    TEMPLATE_MATCHING_THRESHOLD = 0.60 # threshold for template matching
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
            track_object(camera_capture,
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