'''This script uses a pan and tilt servo to control a gimbal with camera attached to it. Once it detects motion, it draws a box around it and uses the cv2.matchTemplate function to find the center of the object. It then moves the gimbal to the center of the object.'''

# import the necessary packages
import time
import cv2

from gopro_webcam import GoProWebcamPlayer
from camera_functions import capture_single_frame, get_cropped_object_image, get_contours, get_largest_contour
from tracking_functions import track_object_steppers


def main():
    '''Main function.'''
    # arduino device is set in stepper_gimbal_functions.py

    # https://github.com/gopro/OpenGoPro/blob/main/demos/python/multi_webcam/multi_webcam/webcam.py
    gopro = GoProWebcamPlayer(serial='646')

    gopro.open()

    # https://gopro.github.io/OpenGoPro/python_sdk/api.html#open_gopro.api.params.WebcamFOV

    gopro.webcam.start(gopro.port, resolution=12, fov=0)

    gopro.player.url = GoProWebcamPlayer.STREAM_URL.format(port=gopro.port)

    camera_capture = cv2.VideoCapture(gopro.player.url + "?overrun_nonfatal=1&fifo_size=500000", cv2.CAP_FFMPEG)

    camera_capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    # wait for the Arduino to initialize
    time.sleep(3)
    print('Arduino initialized')
    print('-'*30)
    print('Sentry Camera Armed')
    print('-'*30)
    
    background_frame = capture_single_frame(camera_capture)

    CONTOUR_THRESHOLD_VALUE = 40.0 # pixel value threshold for contour detection
    MIN_AREA = 10 # minimum area of the contour
    MAX_AREA = 500 # maximum area of the contour
    TEMPLATE_MATCHING_THRESHOLD = 0.80 # threshold for template matching
    OBJECT_BUFFER = 4 # number of pixels to add to each side of the contour when cropping the object
    FRAMES_TO_AVERAGE = 0 # number of frames to average when tracking the object

    number_of_objects = 0 # number of objects detected
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
            if x_of_contour < 5 or y_of_contour < 5:
                print('Contour too small')
                if x_of_contour + width_of_contour > 1920 or y_of_contour + height_of_contour > 1080:
                    print('Contour too close to edge of frame')
                continue
            captured_object = True
            number_of_objects += 1

        # display the frames
        # cv2.imshow("Threshold Frame", threshold_frame)
        cv2.imshow("Background View", current_frame)

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
            # cv2.imshow("Tracking Object (original)", cv2.resize(cropped_object_image, (640, 480)))

            print('Tracking object')

            # switch to tracking object
            switch_to_motion_detection = track_object_steppers(camera_capture,
                                cropped_object_image,
                                template_matching_threshold=TEMPLATE_MATCHING_THRESHOLD,
                                frames_to_average=FRAMES_TO_AVERAGE)
                
            if switch_to_motion_detection:
                print('Finished tracking object')
                print()

                # capture a new background frame
                background_frame = capture_single_frame(camera_capture)
                continue

        if (captured_object and not switch_to_motion_detection) or cv2.waitKey(1) & 0xFF == ord('q'):
            # cleanup
            camera_capture.release()
            cv2.destroyAllWindows()
            gopro.close()
            break


if __name__ == "__main__":
    main()