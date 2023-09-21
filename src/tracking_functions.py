import cv2
import numpy as np

from servo_gimbal_functions import move_pan_servo, move_tilt_servo
from camera_functions import get_cropped_object_image, check_image_match, capture_single_frame, check_image_match_local, template_matching_pytorch
from stepper_gimbal_functions import move_pan_axis, move_tilt_axis

def track_object_servos(camera_capture,
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
        # print('max_val: ', max_val)

        # condition to check object is matched or not
        if max_val > template_matching_threshold:
            crop_object = get_cropped_object_image(frame, max_loc[0], max_loc[1], cropped_object_image.shape[1], cropped_object_image.shape[0])
            all_cropped_objects.append(crop_object)

            # remove the oldest frame if more than 2 objects are stored
            if len(all_cropped_objects) > 2:
                all_cropped_objects.pop(1)

            # take the average of all object images in the list
            cropped_object_image = np.average(all_cropped_objects, axis=0).astype(np.uint8)
        else:
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


def track_object_steppers(camera_capture,
                        cropped_object_image,
                        template_matching_threshold=0.70,
                        frames_to_average=3):
    
    '''Matches the center of the identified object to the center of the camera capture and then uses the difference to move the servos. The next frame is then captured and the process is repeated using the cv2.matchTemplate function.'''

    # create a video writer object for recording the video
    # fourcc = cv2.VideoWriter_fourcc(*'XVID')
    # video_writer = cv2.VideoWriter(filename='output.avi', fourcc=fourcc, fps=24.0, frameSize=(640, 480))

    # Use a list to store the last # frames
    all_cropped_objects = [cropped_object_image]

    first_search = True

    while True:
        # read new frame after cropping the object
        frame = capture_single_frame(camera_capture)

        # perform template matching
        if first_search:
            max_val, max_loc = check_image_match(frame, cropped_object_image)
            first_search = False
        else:
            # search for the template match using a less expensive method
            max_val, max_loc = check_image_match_local(frame, cropped_object_image, max_loc)

        # condition to check object is matched or not
        if max_val > template_matching_threshold:
            crop_object = get_cropped_object_image(frame, max_loc[0], max_loc[1], cropped_object_image.shape[1], cropped_object_image.shape[0])
            all_cropped_objects.append(crop_object)

            # remove the oldest frame if more than _ objects are stored
            if len(all_cropped_objects) > frames_to_average:
                all_cropped_objects.pop(1)

            # take the average of all object images in the list
            cropped_object_image = np.average(all_cropped_objects, axis=0).astype(np.uint8)
        else:
            # video_writer.release()
            cv2.destroyAllWindows()
            return False
            
        # draw a rectangle around the match on a copy of the current frame
        frame_copy = frame.copy()
        cv2.rectangle(frame_copy, max_loc, (max_loc[0] + cropped_object_image.shape[1], max_loc[1] + cropped_object_image.shape[0]), (0, 255, 0), 2)

        # display the frame
        cv2.imshow("Now tracking", frame_copy)

        # display the new cropped object image (debugging)
        # cv2.imshow("Running Average", cv2.resize(cropped_object_image, (640, 480)))

        # calculate the difference between the center of the frame and the center of the match
        difference_x = (frame.shape[1] / 2) - (max_loc[0] + cropped_object_image.shape[1] / 2)
        difference_y = (frame.shape[0] / 2) - (max_loc[1] + cropped_object_image.shape[0] / 2)

        speed = 1500 # microseconds per step (higher = slower)
        steps = 2 # number of steps
        center_threshold = 20 # number of pixels away from center

        # if object outside of deadzone, move the steppers
        if abs(difference_x) > center_threshold:
            if difference_x > 0:
                move_pan_axis('left', speed, steps)
            else:
                move_pan_axis('right', speed, steps)

        if abs(difference_y) > center_threshold:
            if difference_y > 0:
                move_tilt_axis('up', speed, steps)
            else:
                move_tilt_axis('down', speed, steps)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            # cleanup
            camera_capture.release()
            # video_writer.release()
            cv2.destroyAllWindows()
            break


def track_object_steppers_pytorch(camera_capture,
                        cropped_object_image,
                        template_matching_threshold=0.70,
                        frames_to_average=3):
    
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
        y, x = template_matching_pytorch(frame, cropped_object_image) # this is returned at (y, x) instead of (x, y)

        print('x: ', x)
        print('y: ', y)
        print()

        # condition to check object is matched or not
        # if max_val > template_matching_threshold:
        #     crop_object = get_cropped_object_image(frame, max_loc[0], max_loc[1], cropped_object_image.shape[1], cropped_object_image.shape[0])
        #     all_cropped_objects.append(crop_object)

        #     # remove the oldest frame if more than _ objects are stored
        #     if len(all_cropped_objects) > frames_to_average:
        #         all_cropped_objects.pop(1)

        #     # take the average of all object images in the list
        #     cropped_object_image = np.average(all_cropped_objects, axis=0).astype(np.uint8)
        # else:
        #     # video_writer.release()
        #     cv2.destroyAllWindows()
        #     return False
            
        # draw a rectangle around the match on a copy of the current frame
        frame_copy = frame.copy()
        # cv2.rectangle(frame_copy, max_loc, (x + cropped_object_image.shape[0], y + cropped_object_image.shape[1]), (0, 255, 0), 2)

        # display the frame
        cv2.imshow("Now tracking", frame_copy)

        # display the new cropped object image (debugging)
        # cv2.imshow("Running Average", cv2.resize(cropped_object_image, (640, 480)))

        # calculate the difference between the center of the frame and the center of the match
        # difference_x = (frame.shape[1] / 2) - (max_loc[0] + cropped_object_image.shape[1] / 2)
        # difference_y = (frame.shape[0] / 2) - (max_loc[1] + cropped_object_image.shape[0] / 2)

        # speed = 1500 # microseconds per step (higher = slower)
        # steps = 2 # number of steps
        # center_threshold = 20 # number of pixels away from center

        # # if object outside of deadzone, move the steppers
        # if abs(difference_x) > center_threshold:
        #     if difference_x > 0:
        #         move_pan_axis('left', speed, steps)
        #     else:
        #         move_pan_axis('right', speed, steps)

        # if abs(difference_y) > center_threshold:
        #     if difference_y > 0:
        #         move_tilt_axis('up', speed, steps)
        #     else:
        #         move_tilt_axis('down', speed, steps)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            # cleanup
            camera_capture.release()
            # video_writer.release()
            cv2.destroyAllWindows()
            break