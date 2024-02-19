import time
import sys
import tty
import termios

from classes import SerialConnection, Message, MessageCommand, MotorState, MotorDirection, MotorSpeed


def calibrate_steppers(connection: SerialConnection):
    message = Message(MessageCommand.Calibrate, None)
    connection.send(message.dump())
    time.sleep(35)


def set_neutral(connection: SerialConnection):
    message = Message(MessageCommand.SetNeutral, None)
    connection.send(message.dump())
    time.sleep(3)


def move_steppers(connection: SerialConnection, pan: MotorState, tilt: MotorState):
    message = Message(MessageCommand.MoveStepper, (pan, tilt))
    connection.send(message.dump())


def read_keypress() -> int:
    '''Read keypress from the user.'''

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setraw(sys.stdin.fileno())
        keypress = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    return ord(keypress)


def move_gimbal_with_keypress(connection, pan_state, tilt_state):
    '''Manual control of the gimbal using the keyboard.'''

    raw_keypress = read_keypress()
    if raw_keypress == 255:
        return raw_keypress

    if raw_keypress == ord('w'):
        # print('Moving up')
        pan_state.speed = MotorSpeed.Off
        pan_state.direction = MotorDirection.Zero
        tilt_state.speed = MotorSpeed.Speed4
        tilt_state.direction = MotorDirection.Up
    elif raw_keypress == ord('s'):
        # print('Moving down')
        pan_state.speed = MotorSpeed.Off
        pan_state.direction = MotorDirection.Zero
        tilt_state.speed = MotorSpeed.Speed4
        tilt_state.direction = MotorDirection.Down
    elif raw_keypress == ord('a'):
        # print('Moving left')
        pan_state.speed = MotorSpeed.Speed4
        pan_state.direction = MotorDirection.Zero
        tilt_state.speed = MotorSpeed.Off
        tilt_state.direction = MotorDirection.Zero
    elif raw_keypress == ord('d'):
        # print('Moving right')
        pan_state.speed = MotorSpeed.Speed4
        pan_state.direction = MotorDirection.One
        tilt_state.speed = MotorSpeed.Off
        tilt_state.direction = MotorDirection.Zero
    else:
        # print('Stopping')
        pan_state.speed = MotorSpeed.Off
        pan_state.direction = MotorDirection.Zero
        tilt_state.speed = MotorSpeed.Off
        tilt_state.direction = MotorDirection.Zero

    move_steppers(connection, pan_state, tilt_state)
    return raw_keypress


if __name__ == '__main__':
    # initialize serial connection
    connection = SerialConnection()
    print('Serial connection established')

    # add small delay to give the communication a moment to establish
    time.sleep(2)

    # send calibration message
    print('Calibrating steppers')
    calibrate_steppers(connection)
    print('Calibration complete')
    print()

    # initalize motor states
    tilt_state = MotorState()
    pan_state = MotorState()

    while True:
        keypress = move_gimbal_with_keypress(connection, pan_state, tilt_state)
        print('Key pressed: ', keypress)

        if keypress == ord('q'):
            print('Exiting')
            break
