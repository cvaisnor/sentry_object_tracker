'''This script uses a HDMI video feed and gimbal controlled by Visca over IP'''

# import the necessary packages
import time
import cv2
from multiprocessing import Process
from camera_functions import get_cropped_object_image, contour_parser
from tracking_functions import track_object
from visca_over_ip import Camera


def main():
    '''Main function.'''

    camera_capture = cv2.VideoCapture(0)
    gimbal = Camera('10.42.0.37')  # camera IP or hostname

    WIDTH = 1920
    HEIGHT = 1080

    CONTOUR_THRESHOLD_VALUE = 40.0 # pixel value threshold for contour detection
    MIN_AREA = 50 # minimum area of the contour
    MAX_AREA = 2000 # maximum area of the contour
    TEMPLATE_MATCHING_THRESHOLD = 0.70 # threshold for template matching
    OBJECT_BUFFER = 10 # number of pixels to add to each side of the contour when cropping the object
    FRAMES_TO_AVERAGE = 1 # number of frames to average when tracking the object
    GIMBAL_MOVEMENT = True # set to True to track the object with the gimbal

    camera_capture.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    camera_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    camera_capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    assert camera_capture.isOpened(), 'Camera not found'

    # send calibration message
    if GIMBAL_MOVEMENT:
        print('Setting gimbal to home position')
        gimbal.pantilt_home()
        time.sleep(1)

    print('Sentry Camera Armed')
    print('-'*30)
    
    ret, background_frame = camera_capture.read()
    if ret is False:
        print('Error reading first background frame')
        camera_capture.release()
        return

    number_of_objects = 0 # number of objects detected
    while True:
        # capture another frame
        ret, current_frame = camera_capture.read()
        if ret is False:
            print('Error reading frame')
            continue

        # display the background frame
        cv2.imshow("View", current_frame)

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

            # draw a rectangle around the object on a copy of the frame
            current_frame_copy = current_frame.copy()
            cv2.rectangle(current_frame_copy, (x, y), (x+w, y+h), (0, 255, 0), 2)

            # filename to save the captured object image
            filename = f'captured_objects/captured_object_{number_of_objects}.jpg'

            # save the frame with the object rectangle drawn on it
            cv2.imwrite(filename, current_frame_copy)

            # display the cropped object image at a larger size (debugging)
            # cv2.imshow("Tracking Object (original)", cv2.resize(cropped_object_image, (640, 480)))

            # switch to tracking object
            print('Tracking object')
            switch_to_motion_detection = track_object(
                                        gimbal,
                                        camera_capture,
                                        cropped_object_image,
                                        template_matching_threshold=TEMPLATE_MATCHING_THRESHOLD,
                                        frames_to_average=FRAMES_TO_AVERAGE,
                                        number_of_objects=number_of_objects,
                                        gimbal_movement=GIMBAL_MOVEMENT,
                                    )
            
            if switch_to_motion_detection:
                print('Finished tracking object')
                print('Setting gimbal to home position')
                gimbal.pantilt_home()
                time.sleep(2)

                # flush the frames in the buffer
                print('Flushing frames in buffer')
                for i in range(120):
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