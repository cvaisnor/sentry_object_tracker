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