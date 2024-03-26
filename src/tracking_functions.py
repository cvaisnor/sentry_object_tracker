import cv2
import numpy as np
import time

from camera_functions import get_cropped_object_image, check_image_match, check_image_match_local

def track_object(gimbal,
                 pan_pos,
                 tilt_pos,
                 camera_capture,
                 cropped_object_image,
                 template_matching_threshold=0.70,
                 frames_to_average=3,
                 number_of_objects=0,
                 gimbal_movement=False):

    '''Matches the center of the identified object to the center of the camera capture and then uses the difference to move the stepper motors. The next frame is then captured and the process is repeated using the cv2.matchTemplate function.'''

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
            max_val, max_loc = check_image_match_local(frame, cropped_object_image, last_loc=max_loc, obj_padding=200)

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
            # print('Object not found, retrying')
            # first_search = True # change to True to search the entire frame
            search_retry_count += 1 # increment the search retry count

        if search_retry_count > 6: # if the object is not found after 3 retries, stop searching
            # video_writer.release()
            print('Object not found after 3 retries')
            cv2.destroyWindow("Now tracking")
            return True
            
        # draw a rectangle around the match on a copy of the current frame
        frame_copy = frame.copy()
        cv2.rectangle(frame_copy, max_loc, (max_loc[0] + cropped_object_image.shape[1], max_loc[1] + cropped_object_image.shape[0]), (0, 255, 0), 2)

        # display the frame with the rectangle around the match, add text = number of objects detected
        cv2.putText(frame_copy, f'Object #{number_of_objects}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("Now tracking", frame_copy)

        # display the new cropped object image (debugging)
        # cv2.imshow("Running Average", cv2.resize(cropped_object_image, (640, 480)))

        if gimbal_movement:
            # calculate the difference between the center of the frame and the center of the match
            difference_x = (frame.shape[1] / 2) - (max_loc[0] + cropped_object_image.shape[1] / 2)
            difference_y = (frame.shape[0] / 2) - (max_loc[1] + cropped_object_image.shape[0] / 2)

            difference_x = int(difference_x // 100)
            difference_y = int(difference_y // 100)

            center_threshold = 200 # number of pixels away from center

            # use the difference to find the absolute distance to move the gimbal
            pan_speed = 0
            tilt_speed = 0
            
            pan_pos, tilt_pos = gimbal.get_pantilt_position()

            new_pan_pos = pan_pos + difference_x
            new_tilt_pos = tilt_pos + difference_y

            # if object outside of deadzone, move the steppers
            if abs(difference_x) > center_threshold:
                if difference_x > 0: # left
                    pan_speed = 3  
                else: # right
                    # print('Moving right')
                    pan_speed = -3

            if abs(difference_y) > center_threshold:
                if difference_y > 0: # up
                    # print('Moving up')
                    tilt_speed = 3

                else: # down
                    # print('Moving down')
                    tilt_speed = -3

            # if object inside of deadzone, stop the steppers
            if abs(difference_x) < center_threshold: # pan
                # print('no pan')
                pan_speed = 0

            if abs(difference_y) < center_threshold: # tilt
                # print('no tilt')
                tilt_speed = 0

            # move the gimbal
            print(f'New Pan Position: {new_pan_pos}, New Tilt Position: {new_tilt_pos}')
            print()
            gimbal.pantilt(pan_speed, tilt_speed, new_pan_pos, new_tilt_pos)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            # cleanup
            camera_capture.release()
            # video_writer.release()
            cv2.destroyAllWindows()
            return False
