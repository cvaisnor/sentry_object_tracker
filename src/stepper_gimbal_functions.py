import serial
import time

arduino = serial.Serial('/dev/ttyACM0', 9600)

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

    move_tilt_axis('up', 2000, 5)

    time.sleep(0.5)

    move_tilt_axis('down', 2000, 5)

    time.sleep(0.5)

    move_pan_axis('left', 2000, 5)

    time.sleep(0.5)

    move_pan_axis('right', 2000, 5)

