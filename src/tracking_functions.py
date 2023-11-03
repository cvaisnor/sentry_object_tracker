import cv2
import numpy as np

from camera_functions import get_cropped_object_image, check_image_match, check_image_match_local
from servo_gimbal_functions import move_pan_servo, move_tilt_servo


def track_object_servos(camera_capture,
                        cropped_object_image,
                        gimbal,
                        pan_servo,
                        tilt_servo,
                        pan_servo_range,
                        tilt_servo_range,
                        template_matching_threshold,
                        frames_to_average,
                        number_of_objects,
                        gimbal_movement):

    '''Matches the center of the identified object to the center of the camera capture and then uses the difference to move the servos. The next frame is then captured and the process is repeated using the cv2.matchTemplate function.'''

    # create a video writer object for recording the video
    # fourcc = cv2.VideoWriter_fourcc(*'XVID')
    # video_writer = cv2.VideoWriter(filename='output.avi', fourcc=fourcc, fps=24.0, frameSize=(640, 480))

    # Use a list to store the last # frames
    all_cropped_objects = [cropped_object_image]

    first_search = True

    search_retry_count = 0

    while True:
        # read new frame after cropping the object
        ret, frame = camera_capture.read()
        if ret is False:
            print('Error reading frame')
            continue

        # perform template matching
        if first_search:
            max_val, max_loc = check_image_match(frame, cropped_object_image)
            first_search = False
        else:
            # search for the template match using a less expensive method
            max_val, max_loc = check_image_match_local(frame, cropped_object_image, last_loc=max_loc, obj_padding=50)

        # condition to check object is matched or not
        if max_val > template_matching_threshold:
            crop_object = get_cropped_object_image(frame, max_loc[0], max_loc[1], cropped_object_image.shape[1], cropped_object_image.shape[0])
            all_cropped_objects.append(crop_object)
            search_retry_count = 0 # reset the search retry count

            # remove the oldest frame if more than _ objects are stored
            if len(all_cropped_objects) > frames_to_average:
                all_cropped_objects.pop(1)

            # take the average of all object images in the list
            cropped_object_image = np.average(all_cropped_objects, axis=0).astype(np.uint8)
        elif max_val < template_matching_threshold - 0.1:
            print('Object not found, retrying')
            # first_search = True # change to True to search the entire frame
            search_retry_count += 1 # increment the search retry count

        if search_retry_count > 6: # if the object is not found after # retries, stop searching
            # video_writer.release()
            print('Object not found after 3 retries')
            cv2.destroyAllWindows()
            return True
            
        # draw a rectangle around the match on a copy of the current frame
        frame_copy = frame.copy()
        cv2.rectangle(frame_copy, max_loc, (max_loc[0] + cropped_object_image.shape[1], max_loc[1] + cropped_object_image.shape[0]), (0, 255, 0), 2)

        # display the frame with the rectangle around the match, add text = number of objects detected
        cv2.putText(frame_copy, f'Object #{number_of_objects}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # display the new cropped object image (debugging)
        # cv2.imshow("Running Average", cv2.resize(cropped_object_image, (640, 480)))

        if gimbal_movement:
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
            return False