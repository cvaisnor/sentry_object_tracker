def set_servos_neutral(gimbal, pan_servo, tilt_servo, pan_servo_range, tilt_servo_range):
    '''Sets the servos to the neutral position.'''
    # neutral position is the middle of the range
    pan_neutral = int((pan_servo_range[1] + pan_servo_range[0]) / 2)
    tilt_neutral = int((tilt_servo_range[1] + tilt_servo_range[0]) / 2)
    gimbal.servo[pan_servo].angle = pan_neutral
    gimbal.servo[tilt_servo].angle = tilt_neutral


def move_servo_to_position(gimbal, servo, position, servo_range):
    '''Moves the servo to the specified angle.'''
    if position in range(servo_range[0], servo_range[1]):
        gimbal.servo[servo].angle = position
    else:
        print("Position out of range")
        print("Servo: ", servo)
        print("Position: ", position)
    return


def move_servo_by_degrees(gimbal, servo, degrees, servo_range):
    '''Moves the servo by the specified number of degrees.'''
    # get the current position
    current_position = int(gimbal.servo[servo].angle)
    # calculate the new position
    new_position = current_position + degrees
    if new_position in range(servo_range[0], servo_range[1]):
        gimbal.servo[servo].angle += degrees
    else:
        print("Position out of range")
        print("Servo: ", servo)
        print("New Position: ", new_position)
        print('Range: ', servo_range)
    return


def move_gimbal_by_difference(gimbal, pan_servo, tilt_servo, difference_x, difference_y, pan_servo_range, tilt_servo_range, servo_adjustment_speed):
    '''Moves the gimbal by the difference between the center of the template match and the center of the camera capture.'''
    # move the pan servo
    if difference_x > 0:
        # print('Moving left')
        # move left
        move_servo_by_degrees(gimbal, pan_servo, servo_adjustment_speed, pan_servo_range)
    elif difference_x < 0:
        # print('Moving right')
        # move right
        move_servo_by_degrees(gimbal, pan_servo, -servo_adjustment_speed, pan_servo_range)

    # move the tilt servo
    if difference_y > 0:
        # print('Moving up')
        # move up
        move_servo_by_degrees(gimbal, tilt_servo, servo_adjustment_speed, tilt_servo_range)
    elif difference_y < 0:
        # print('Moving down')
        # move down
        move_servo_by_degrees(gimbal, tilt_servo, -servo_adjustment_speed, tilt_servo_range)


def move_pan_servo(gimbal, pan_servo, pan_servo_range, servo_adjustment_speed, difference_x):
    '''Moves the pan servo by the difference in the x direction.'''
    if difference_x > 0:
        # print('Moving left')
        # move left
        move_servo_by_degrees(gimbal, pan_servo, servo_adjustment_speed, pan_servo_range)
    elif difference_x < 0:
        # print('Moving right')
        # move right
        move_servo_by_degrees(gimbal, pan_servo, -servo_adjustment_speed, pan_servo_range)


def move_tilt_servo(gimbal, tilt_servo, tilt_servo_range, servo_adjustment_speed, difference_y):
    '''Moves the tilt servo by the difference in the y direction.'''
    if difference_y > 0:
        # print('Moving up')
        # move up
        move_servo_by_degrees(gimbal, tilt_servo, servo_adjustment_speed, tilt_servo_range)
    elif difference_y < 0:
        # print('Moving down')
        # move down
        move_servo_by_degrees(gimbal, tilt_servo, -servo_adjustment_speed, tilt_servo_range)


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
            return True
            
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