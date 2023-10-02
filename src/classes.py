import serial
import time
from enum import IntEnum, auto

global degrees_per_step

stepper_gear_size = 20 # teeth

pan_gear_size = 80 # teeth
tilt_gear_size = 60 # teeth

degrees_per_step = 1.8 # degrees of rotation per step on stepper motor

pan_gear_ratio = pan_gear_size / stepper_gear_size
tilt_gear_ratio = tilt_gear_size / stepper_gear_size

pan_degrees_per_step = degrees_per_step / pan_gear_ratio
tilt_degrees_per_step = degrees_per_step / tilt_gear_ratio


def calibrate_gimbal(SerialConnection):
    # letter 
    pass
    

def set_gimbal_neutral():
    pass


def move_pan(steps, direction, speed):
    pass


def move_tilt(steps, direction, speed):
    pass


def set_gimbal_state(pan_command, tilt_command):
    pass


class SerialConnection():
    def __init__(self) -> None:
        self.baudrate = 9600
        self.port = '/dev/ttyACM0'
        self.timeout = 2
        self.arduino = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
    
    def send(self, message):
        self.arduino.write(message)

    
    def receive(self):
        return self.arduino.readline()


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


class MessageCommand(IntEnum):
    Calibrate = 0
    SetNeutral = 1
    MoveStepper = 2


class MotorState:
    def __init__(self, direction = MotorDirection.Zero, speed = MotorSpeed.Off):
        self.direction = direction
        self.speed = speed
        

class Message():
    '''
    2 byte message to be sent to the Arduino
    first byte is Calibrate, SetNeutral, or MoveStepper

    Second byte:
    0x00: Pan Direction
    0x01: Tilt Direction
    0x02: PanSpeed
    0x03: PanSpeed
    0x04: PanSpeed
    0x05: TiltSpeed
    0x06: TiltSpeed
    0x07: TiltSpeed
    '''
    def __init__(self, message: MessageCommand, data) -> None:
        self.message = message
        self.data = data

    
    def dump(self):
        message = bytes([self.message])
        if self.data:
            # convert data to bytes
            (pan, tilt) = self.data

            state = (pan.direction & 0b1) << 7
            state |= (tilt.direction & 0b1) << 6
            state |= (pan.speed & 0b111) << 3
            state |= tilt.speed & 0b111

            message += bytes([state])
        return message
    
if __name__ == '__main__':
    # initialize serial connection
    connection = SerialConnection()

    # add small delay to give the communication a moment to establish
    time.sleep(2)

    # send calibration message
    message = Message(MessageCommand.Calibrate, None)
    connection.send(message.dump())

    time.sleep(2)