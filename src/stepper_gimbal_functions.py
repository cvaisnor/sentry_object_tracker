from enum import auto, IntEnum
import serial
import time

arduino = serial.Serial('/dev/ttyACM0', 9600)

class MotorDirection(IntEnum):
    Zero = 0
    One = 1
    Up = 1
    Down = 0
    Left = 0
    Right = 1

class MotorSpeed(IntEnum):
    Off = 0
    Speed1 = 1
    Speed2 = 2
    Speed3 = 3
    Speed4 = 4
    Speed5 = 5
    Speed6 = 6
    Speed7 = 7

class MotorState:
    def __init__(self, direction = MotorDirection.Zero, speed = MotorSpeed.Off):
        self.direction = direction
        self.speed = speed

def set_gimbal_state(pan = MotorState(), tilt = MotorState()):
    state = (pan.direction & 0b1) << 7
    state |= (tilt.direction & 0b1) << 6
    state |= (pan.speed & 0b111) << 3
    state |= tilt.speed & 0b111

    arduino.write(bytes([state]))

    # print the message as a string of bits
    # print(f'Sending command: {state:08b}')


if __name__ == '__main__':

    # add small delay to give the communication a moment to establish
    time.sleep(2)

    print('Testing gimbal motors...')
    set_gimbal_state(tilt=MotorState(direction=MotorDirection.Up, speed=MotorSpeed.Speed1))
    time.sleep(0.5)

    set_gimbal_state(tilt=MotorState(direction=MotorDirection.Down, speed=MotorSpeed.Speed1))
    time.sleep(0.5)

    set_gimbal_state(pan=MotorState(direction=MotorDirection.Left, speed=MotorSpeed.Speed1))
    time.sleep(0.5)

    set_gimbal_state(pan=MotorState(direction=MotorDirection.Right, speed=MotorSpeed.Speed1))
    time.sleep(0.5)