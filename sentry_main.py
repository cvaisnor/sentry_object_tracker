'''This script uses a pan and tilt servo to control a gimbal with camera attached to it. Once it detects motion, it draws a box around it and uses the cv2.matchTemplate function to find the center of the object. It then moves the gimbal to the center of the object.'''

# import the necessary packages
import time
import cv2
import board
import busio

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

    while True:
        # read new frame after cropping the object
        frame = capture_single_frame(camera_capture)
        # video_writer.write(frame)

        # perform template matching
        max_val, max_loc = check_image_match(frame, cropped_object_image)
        if max_val < template_matching_threshold:
            print('No match found')
            cv2.destroyAllWindows()
            return False
            
        # draw a rectangle around the match in the current frame
        cv2.rectangle(frame, max_loc, (max_loc[0] + cropped_object_image.shape[1], max_loc[1] + cropped_object_image.shape[0]), (0, 255, 0), 2)

        # display the frame
        cv2.imshow("Now tracking", frame)

        # calculate the difference between the center of the frame and the center of the match
        difference_x = (frame.shape[1] / 2) - (max_loc[0] + cropped_object_image.shape[1] / 2)
        difference_y = (frame.shape[0] / 2) - (max_loc[1] + cropped_object_image.shape[0] / 2)

        # the larger the difference, the faster the servos move
        servo_adjustment_speed = int(abs(difference_x) / 14) + 1
        if servo_adjustment_speed > 7:
            servo_adjustment_speed = 7 # limit the speed

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

    CONTOUR_THRESHOLD_VALUE = 60.0 # pixel value threshold for contour detection
    MIN_AREA = 100 # minimum area of the contour
    MAX_AREA = 1000 # maximum area of the contour
    TEMPLATE_MATCHING_THRESHOLD = 0.60 # threshold for template matching

    while True:

        captured_object = False

        # capture another frame
        current_frame = capture_single_frame(camera_capture)

        contours, threshold_frame, difference_frame = get_contours(background_frame, current_frame, threshold_value=CONTOUR_THRESHOLD_VALUE)

        if len(contours) > 0:
            largest_countour = get_largest_contour(contours, min_area=MIN_AREA, max_area=MAX_AREA) # find the object with the largest contour
            
            # get the x, y, width, and height of box around the contour
            x_of_contour, y_of_contour, width_of_contour, height_of_contour = cv2.boundingRect(largest_countour)

            if width_of_contour > 0 and height_of_contour > 0:
                # if the contour is too close to the edge of the frame, ignore it
                if x_of_contour < 20 or y_of_contour < 20 or x_of_contour + width_of_contour > 620 or y_of_contour + height_of_contour > 460:
                    print('Contour too close to edge of frame')
                    continue
                captured_object = True

        # display the frames
        cv2.imshow("Threshold Frame", threshold_frame)
        cv2.imshow("Motion Detection", current_frame)

        if captured_object:
            cropped_object_image = get_cropped_object_image(current_frame, x_of_contour, y_of_contour, width_of_contour, height_of_contour) # crop the object from the current frame

            # display the cropped object image at a larger size (debugging)
            # cv2.imshow("Tracking Object", cv2.resize(cropped_object_image, (640, 480)))

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