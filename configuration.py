'''This script configures the gimbal for the video capture and motion detection scripts.'''
from classes import Servo, Static_Camera, Gimbal
from adafruit_servokit import ServoKit
import cv2


# set of functions to move the gimbal using the arrow keys

# read the keypresses
def read_keypress():
    key = cv2.waitKey(1) & 0xFF
    return key


def move_gimbal(gimbal, key):
    if key == ord('w'):
        gimbal.move_tilt_up(1)
    elif key == ord('s'):
        gimbal.move_tilt_down(1)
    elif key == ord('a'):
        gimbal.move_pan_left(1)
    elif key == ord('d'):
        gimbal.move_pan_right(1)
    elif key == ord('q'):
        gimbal.move_pan_left(1)
        gimbal.move_tilt_up(1)
    elif key == ord('e'):
        gimbal.move_pan_right(1)
        gimbal.move_tilt_up(1)
    elif key == ord('z'):
        gimbal.move_pan_left(1)
        gimbal.move_tilt_down(1)
    elif key == ord('x'):
        gimbal.move_pan_right(1)
        gimbal.move_tilt_down(1)
    elif key == ord('n'):
        gimbal.set_neutral()

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


def main():
    # Create an object to access the ServoKit library
    gimbal = ServoKit(channels=16) # using PCA9685

    pan_servo = 0 # channel 0
    pan_servo_range = [0, 180]

    tilt_servo = 1 # channel 1
    tilt_servo_range = [90, 160]

    while True:

        # read the keypresses
        key = read_keypress()
        if key == ord('q'):
            break
        else:
            move_gimbal(gimbal, key)