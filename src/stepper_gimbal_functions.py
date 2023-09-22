from enum import auto, IntEnum
import serial
import time

arduino = serial.Serial('/dev/ttyACM0', 9600)

class MotorDirection(IntEnum):
    Zero = 0
    One = 1
    Down = 0
    Up = 1
    Left = 0
    Right = 1

class MotorSpeed(IntEnum):
    Off = 0
    Speed1 = auto()
    Speed2 = auto()
    Speed3 = auto()
    Speed4 = auto()
    Speed5 = auto()
    Speed6 = auto()
    Speed7 = auto()

class MotorState:
    def __init__(self, direction = MotorDirection.Zero, speed = MotorSpeed.Off):
        self.direction = direction
        self.speed = speed

def set_gimbal_state(pan = MotorState(), tilt = MotorState()):
    state = (pan.direction & 0b1) << 7
    state &= (tilt.direction & 0b1) << 6
    state &= (pan.speed & 0b111) << 3
    state &= tilt.speed & 0b111

    arduino.write(bytes([state]))

def move_tilt_axis(direction, speed, distance):
    axis_id = '1'  # change the axis id to string
    direction = '1' if direction == 'up' else '0'  # change the torque to string
    speed = str(speed)  # convert speed to string
    distance = str(distance)  # convert distance to string
    # create the command string using str.format()
    command = '{} {} {} {}\n'.format(axis_id, direction, speed, distance)

    # display the command string
    # print(f'Sending command: {command}')
    # send the command to the Arduino
    arduino.write(command.encode()) # encode the command string to bytes


def move_pan_axis(direction, speed, distance):
    axis_id = '0'  # change the axis id to string
    direction = '0' if direction == 'left' else '1'  # change the torque to string
    speed = str(speed)  # convert speed to string
    distance = str(distance)  # convert distance to string
    # create the command string using str.format()
    command = '{} {} {} {}\n'.format(axis_id, direction, speed, distance)

    # display the command string
    # print(f'Sending command: {command}')
    # send the command to the Arduino
    arduino.write(command.encode()) # encode the command string to bytes


if __name__ == '__main__':

    # add small delay to give the communication a moment to establish
    time.sleep(3)

    set_gimbal_state(tilt=MotorState(direction=MotorDirection.Up, speed=MotorSpeed.Speed1))
    time.sleep(0.5)

    set_gimbal_state(tilt=MotorState(direction=MotorDirection.Down, speed=MotorSpeed.Speed1))
    time.sleep(0.5)

    set_gimbal_state(pan=MotorState(direction=MotorDirection.Left, speed=MotorSpeed.Speed1))
    time.sleep(0.5)

    set_gimbal_state(pan=MotorState(direction=MotorDirection.Right, speed=MotorSpeed.Speed1))
    time.sleep(0.5)

    # move_tilt_axis('up', 2000, 5)
    # time.sleep(0.5)

    # move_tilt_axis('down', 2000, 5)
    # time.sleep(0.5)

    # move_pan_axis('left', 2000, 5)
    # time.sleep(0.5)

    # move_pan_axis('right', 2000, 5)

