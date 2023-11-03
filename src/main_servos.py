'''This script uses a pan and tilt servo to control a gimbal with camera attached to it. Once it detects motion, it draws a box around it and uses the cv2.matchTemplate function to find the center of the object. It then moves the gimbal to the center of the object.'''

# import the necessary packages
import cv2
import board
import busio
from adafruit_servokit import ServoKit

from camera_functions import get_cropped_object_image, contour_parser
from servo_gimbal_functions import set_servos_neutral
from tracking_functions import track_object_servos


def main():
    '''Main function.'''

    # Create an I2C device
    i2c = busio.I2C(board.SCL, board.SDA)

    # create a kit object
    gimbal = ServoKit(channels=16, i2c=i2c)

    pan_servo = 0 # channel 0
    pan_servo_range = [0, 180]

    tilt_servo = 1 # channel 1
    tilt_servo_range = [0, 180]

    # set the servos to the neutral position
    print('Setting gimbal to neutral position')
    set_servos_neutral(gimbal, pan_servo, tilt_servo, pan_servo_range, tilt_servo_range)
    print()

    camera_capture = cv2.VideoCapture(0)
    WIDTH = 640
    HEIGHT = 480

    camera_capture.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    camera_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    camera_capture.set(cv2.CAP_PROP_FPS, 30)
    camera_capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    print('Sentry Camera Armed')
    print('-'*30)
    
    ret, background_frame = camera_capture.read()
    if ret is False:
        print('Error reading first background frame')
        return

    CONTOUR_THRESHOLD_VALUE = 40.0 # pixel value threshold for contour detection
    MIN_AREA = 5 # minimum area of the contour
    MAX_AREA = 1000 # maximum area of the contour
    TEMPLATE_MATCHING_THRESHOLD = 0.80 # threshold for template matching
    OBJECT_BUFFER = 0 # number of pixels to add to each side of the contour when cropping the object
    FRAMES_TO_AVERAGE = 4 # number of frames to average when tracking the object
    GIMBAL_MOVEMENT = True # set to True to track the object with the gimbal

    number_of_objects = 0 # number of objects detected
    while True:
        # capture another frame
        ret, current_frame = camera_capture.read()
        if ret is False:
            print('Error reading frame')
            continue

        # display the background frame
        cv2.imshow("View", background_frame)

        contour_found, x, y, w, h = contour_parser(current_frame,
                                                    background_frame,
                                                    CONTOUR_THRESHOLD_VALUE,
                                                    MIN_AREA,
                                                    MAX_AREA,
                                                    frame_width=WIDTH,
                                                    frame_height=HEIGHT)

        if contour_found:
            number_of_objects += 1
            # add pixels to each side of the contour to get a buffer
            x -= OBJECT_BUFFER
            y -= OBJECT_BUFFER
            w += 2*OBJECT_BUFFER
            h += 2*OBJECT_BUFFER

            cropped_object_image = get_cropped_object_image(current_frame, x, y, w, h) # crop the object from the current frame

            # filename to save the captured object image
            filename = f'captured_objects/captured_object_{number_of_objects}.jpg'

            # save the cropped object image to /captured_objects
            cv2.imwrite(filename, cropped_object_image)

            # display the cropped object image at a larger size (debugging)
            # cv2.imshow("Tracking Object (original)", cv2.resize(cropped_object_image, (640, 480)))

            print('Tracking object')

            # switch to tracking object
            switch_to_motion_detection = track_object_servos(
                                            camera_capture,
                                            cropped_object_image,
                                            gimbal,
                                            pan_servo,
                                            tilt_servo,
                                            pan_servo_range,
                                            tilt_servo_range,
                                            template_matching_threshold=TEMPLATE_MATCHING_THRESHOLD,
                                            frames_to_average=FRAMES_TO_AVERAGE,
                                            number_of_objects=number_of_objects,
                                            gimbal_movement=GIMBAL_MOVEMENT
                                        )
            

            if switch_to_motion_detection:
                print('Finished tracking object')
                print('Setting gimbal to neutral position')
                # set the gimbal to neutral position
                set_servos_neutral(gimbal, pan_servo, tilt_servo, pan_servo_range, tilt_servo_range)
                print()

                # flush the frames in the buffer
                print('Flushing frames in buffer')
                for i in range(30):
                    camera_capture.grab()

                # capture a new background frame
                ret, background_frame = camera_capture.read()
                print('New background frame captured')
                continue

        if (contour_found and not switch_to_motion_detection) or cv2.waitKey(1) & 0xFF == ord('q'):
            # cleanup
            camera_capture.release()
            cv2.destroyAllWindows()
            print('Closing Program')
            break


if __name__ == "__main__":
    main()