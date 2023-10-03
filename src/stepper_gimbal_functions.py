import time
import cv2

from classes import SerialConnection, Message, MessageCommand, MotorState, MotorDirection, MotorSpeed


def calibrate_steppers(connection: SerialConnection):
    message = Message(MessageCommand.Calibrate, None)
    connection.send(message.dump())
    time.sleep(16)


def set_neutral(connection: SerialConnection):
    message = Message(MessageCommand.SetNeutral, None)
    connection.send(message.dump())
    time.sleep(3)


def move_steppers(connection: SerialConnection, pan: MotorState, tilt: MotorState):
    message = Message(MessageCommand.MoveStepper, (pan, tilt))
    connection.send(message.dump())


def read_keypress() -> int:
    '''Read keypress from the user.'''
    keypress = cv2.waitKey(1) & 0xFF
    return keypress


def move_gimbal_with_keypress(connection, pan_state, tilt_state):
    '''Manual control of the gimbal using the keyboard.'''
    keypress = 255

    keypress = read_keypress()
    if keypress == 255:
        return keypress

    if keypress == ord('w'):
        # print('Moving up')
        tilt_state.speed = MotorSpeed.Speed4
        tilt_state.direction = MotorDirection.Up
    elif keypress == ord('s'):
        # print('Moving down')
        tilt_state.speed = MotorSpeed.Speed4
        tilt_state.direction = MotorDirection.Down
    elif keypress == ord('a'):
        # print('Moving left')
        pan_state.speed = MotorSpeed.Speed4
        pan_state.direction = MotorDirection.Zero
    elif keypress == ord('d'):
        # print('Moving right')
        pan_state.speed = MotorSpeed.Speed4
        pan_state.direction = MotorDirection.One
    else:
        # print('Stopping')
        pan_state.speed = MotorSpeed.Off
        pan_state.direction = MotorDirection.Zero
        tilt_state.speed = MotorSpeed.Off
        tilt_state.direction = MotorDirection.Zero

    move_steppers(connection, pan_state, tilt_state)
    return keypress


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

    # move tilt axis up
    print('Moving tilt axis up')
    for i in range(0, 5):
        tilt_state = MotorState(MotorDirection.Up, MotorSpeed.Speed4)
        move_steppers(connection, pan_state, tilt_state)
    print('Motion complete')
    print()
    time.sleep(2)

    # set neutral position
    print('Setting neutral position')
    set_neutral(connection)
    print('Neutral position set')

    # while True:
    
    #     keypress = move_gimbal_with_keypress(connection, pan_state, tilt_state)
    #     print('Key pressed: ', keypress)

    #     if cv2.waitKey(1) & 0xFF == ord('q') or keypress == ord('q'):
    #         # cleanup the camera and close any open windows
    #         cv2.destroyAllWindows()
    #         break