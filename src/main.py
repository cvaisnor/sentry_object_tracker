'''This script uses a pan and tilt servo to control a gimbal with camera attached to it. Once it detects motion, it draws a box around it and uses the cv2.matchTemplate function to find the center of the object. It then moves the gimbal to the center of the object.'''

# import the necessary packages
import time
import cv2

from camera_functions import get_cropped_object_image, get_contours, get_largest_contour
from tracking_functions import track_object


def main():
    '''Main function.'''
    # arduino device is set in stepper_gimbal_functions.py

    camera_capture = cv2.VideoCapture(0)

    WIDTH = 1920
    HEIGHT = 1080

    camera_capture.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    camera_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    camera_capture.set(cv2.CAP_PROP_FPS, 60)
    camera_capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    # wait for the Arduino to initialize
    time.sleep(3)
    print('Arduino initialized')
    print('-'*30)
    print('Sentry Camera Armed')
    print('-'*30)
    
    ret, background_frame = camera_capture.read()

    time.sleep(2)

    CONTOUR_THRESHOLD_VALUE = 40.0 # pixel value threshold for contour detection
    MIN_AREA = 5 # minimum area of the contour
    MAX_AREA = 1000 # maximum area of the contour
    TEMPLATE_MATCHING_THRESHOLD = 0.80 # threshold for template matching
    OBJECT_BUFFER = 0 # number of pixels to add to each side of the contour when cropping the object
    FRAMES_TO_AVERAGE = 4 # number of frames to average when tracking the object
    GIMBAL_MOVEMENT = True # set to True to track the object with the gimbal

    number_of_objects = 0 # number of objects detected
    while True:

        captured_object = False

        # capture another frame
        ret, current_frame = camera_capture.read()
        if ret is False:
            print('Error reading frame')
            continue

        contours, threshold_frame, difference_frame = get_contours(background_frame, current_frame, threshold_value=CONTOUR_THRESHOLD_VALUE)

        current_frame_copy = current_frame.copy()

        if len(contours) > 0:
            largest_countour = get_largest_contour(contours, min_area=MIN_AREA, max_area=MAX_AREA) # find the object with the largest contour
            
            if largest_countour is None:
                print('No contours found that meet the area requirements.')
                continue

            # get the x, y, width, and height of box around the contour
            x_of_contour, y_of_contour, width_of_contour, height_of_contour = cv2.boundingRect(largest_countour)

            # draw a box around the contour
            cv2.rectangle(current_frame_copy, (x_of_contour, y_of_contour), (x_of_contour + width_of_contour, y_of_contour + height_of_contour), (0, 255, 0), 2)

            if x_of_contour + width_of_contour > WIDTH or y_of_contour + height_of_contour > HEIGHT: # if the contour is too close to the edge of the frame, ignore it
                print('Contour too close to edge of frame')
                continue
            captured_object = True
            number_of_objects += 1

        # display the frames
        cv2.imshow("Threshold Frame", threshold_frame)
        cv2.imshow("Background View", current_frame_copy)

        if captured_object:
            # add pixels to each side of the contour to get a buffer
            x_of_contour -= OBJECT_BUFFER
            y_of_contour -= OBJECT_BUFFER
            width_of_contour += 2*OBJECT_BUFFER
            height_of_contour += 2*OBJECT_BUFFER

            cropped_object_image = get_cropped_object_image(current_frame, x_of_contour, y_of_contour, width_of_contour, height_of_contour) # crop the object from the current frame

            # filename to save the captured object image
            filename = f'captured_objects/captured_object_{number_of_objects}.jpg'

            # save the cropped object image to /captured_objects
            cv2.imwrite(filename, cropped_object_image)

            # display the cropped object image at a larger size (debugging)
            cv2.imshow("Tracking Object (original)", cv2.resize(cropped_object_image, (640, 480)))

            print('Tracking object')

            # switch to tracking object
            switch_to_motion_detection = track_object(camera_capture,
                                cropped_object_image,
                                template_matching_threshold=TEMPLATE_MATCHING_THRESHOLD,
                                frames_to_average=FRAMES_TO_AVERAGE,
                                number_of_objects=number_of_objects,
                                gimbal_movement=GIMBAL_MOVEMENT)
                
            if switch_to_motion_detection:
                print('Finished tracking object')
                print()

                # capture a new background frame
                ret, background_frame = camera_capture.read()
                continue

        if (captured_object and not switch_to_motion_detection) or cv2.waitKey(1) & 0xFF == ord('q'):
            # cleanup
            camera_capture.release()
            cv2.destroyAllWindows()
            print('Closing Program')
            # gopro.close()
            break


if __name__ == "__main__":
    main()