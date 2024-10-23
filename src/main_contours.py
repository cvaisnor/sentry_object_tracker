# import the necessary packages
import time
import cv2
import atexit

from multiprocessing import Process
from gimbal_command_process import Gimbal
from classes import SerialConnection, SerialMessagesQueue
from tracking_functions import track_object_visca, track_object_serial
from stepper_gimbal_functions import calibrate_steppers, set_neutral
from camera_functions import get_cropped_object_image, contour_parser, create_trackbar_window, read_trackbar_values, get_contours

def main():
    '''Main function.'''

    def close_program():
        camera_capture.release()
        cv2.destroyAllWindows()
        if SERIAL:
            serial_queue_process.terminate()
        else: 
            gimbal_process.close_gimbal()

    SERIAL = True

    camera_capture = cv2.VideoCapture(0)
    camera_capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG')) # codec
    WIDTH = camera_capture.get(cv2.CAP_PROP_FRAME_WIDTH)
    HEIGHT = camera_capture.get(cv2.CAP_PROP_FRAME_HEIGHT)

    if SERIAL:
        ### Start Serial
        connection = SerialConnection() # set port in this class
        time.sleep(2)
        print('Calibrating...')
        calibrate_steppers(connection)
        print('Calibration complete')
        serial_queue = SerialMessagesQueue(connection)
        serial_queue_process = Process(target=serial_queue.start, args=())
        serial_queue_process.start()
        print('Serial queue process started')
        print('-'*30)
        ### End Serial
    else:
        ### Start VISCA
        print('Starting gimbal command process')
        camera_ip = '10.42.0.37' # modify for your camera
        gimbal_process = Gimbal(ip_address=camera_ip)
        gimbal_process.go_to_home()
        ### End VISCA

    assert camera_capture.isOpened(), 'Camera not found'

    create_trackbar_window()
    atexit.register(close_program)

    print('Sentry Camera Armed')
    print('-'*30)
    
    ret, background_frame = camera_capture.read()
    if ret is False:
        print('Error reading first background frame')
        camera_capture.release()
        if SERIAL:
            serial_queue_process.terminate()
        return

    number_of_objects = 0 # number of objects detected

    while True:
        contour_threshold_value, min_area, max_area, template_matching_threshold, pixel_buffer, frames_to_average, gimbal_movement = read_trackbar_values()

        ret, current_frame = camera_capture.read() # capture a frame
        if ret is False:
            print('Error reading frame')
            continue

        current_frame_copy = current_frame.copy() # copy the frame to draw on
        current_frame_copy_object = current_frame.copy() # copy the frame to draw on

        # showing parameters on the frame
        cv2.rectangle(current_frame_copy, (int(current_frame.shape[1]/2 - min_area/2), int(current_frame.shape[0]/2 - min_area/2)), (int(current_frame.shape[1]/2 + min_area/2), int(current_frame.shape[0]/2 + min_area/2)), (0, 255, 0), 2)
        cv2.rectangle(current_frame_copy, (int(current_frame.shape[1]/2 - max_area/2), int(current_frame.shape[0]/2 - max_area/2)), (int(current_frame.shape[1]/2 + max_area/2), int(current_frame.shape[0]/2 + max_area/2)), (0, 0, 255), 2)

        contours, threshold_frame, difference_frame = get_contours(background_frame, current_frame, threshold_value=contour_threshold_value)
        
        # display the frames
        cv2.imshow("View", current_frame_copy)
        cv2.imshow("Threshold", cv2.resize(threshold_frame, (int(WIDTH/2), int(HEIGHT/2)))) # half size
        cv2.imshow("Difference", cv2.resize(difference_frame, (int(WIDTH/2), int(HEIGHT/2)))) # half size

        contour_found, x, y, w, h = contour_parser(contours, min_area, max_area, frame_width=WIDTH, frame_height=HEIGHT)
        
        if contour_found:
            number_of_objects += 1
            # add pixels to each side of the contour to get a buffer
            x -= pixel_buffer; y -= pixel_buffer; w += 2*pixel_buffer; h += 2*pixel_buffer

            cropped_object_image = get_cropped_object_image(current_frame, x, y, w, h) # crop the object from the current frame

            cv2.rectangle(current_frame_copy_object, (x, y), (x+w, y+h), (0, 255, 0), 2) # rectangle around the object in saved frame

            filename = f'captured_objects/captured_object_{number_of_objects}.jpg' # filename to save the captured object image

            # save the frame with the object rectangle drawn on it
            cv2.imwrite(filename, current_frame_copy_object)

            # switch to tracking object
            print('Tracking object')
            if SERIAL:
                switch_to_motion_detection = track_object_serial(
                                            connection,
                                            camera_capture,
                                            cropped_object_image,
                                            template_matching_threshold=template_matching_threshold,
                                            frames_to_average=frames_to_average,
                                            number_of_objects=number_of_objects,
                                            gimbal_movement=gimbal_movement,
                                            serial_queue=serial_queue
                                        )
                
            else:
                switch_to_motion_detection = track_object_visca(
                                            gimbal_process,
                                            camera_capture,
                                            cropped_object_image,
                                            template_matching_threshold=template_matching_threshold,
                                            frames_to_average=frames_to_average,
                                            number_of_objects=number_of_objects,
                                            gimbal_movement=gimbal_movement,
                                        )
                
            if switch_to_motion_detection:
                print('Finished tracking object')
                if gimbal_movement:
                    print('Setting gimbal to home position')
                    if SERIAL:
                        set_neutral(connection)
                    else:
                        gimbal_process.go_to_home()

                # flush the frames in the buffer
                print('Flushing frames in buffer')
                for i in range(120):
                    camera_capture.grab()
                
                # capture a new background frame
                ret, background_frame = camera_capture.read()
                print('New background frame captured')
                print()
                continue

        if (contour_found and not switch_to_motion_detection) or cv2.waitKey(1) & 0xFF == ord('q'):
            # cleanup
            print('Closing Program')
            break


if __name__ == "__main__":
    main()