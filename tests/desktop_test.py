'''This file is used to make sure the FT232H is working properly and can communicate with the servo PCA9685 servo driver.'''

# from pyftdi.ftdi import Ftdi
import board
import busio
from adafruit_servokit import ServoKit

# ftdi.show_devices() returns ftdi://ftdi:232h:1:9/1
# ftdi = Ftdi()

# SCL is on pin AD0
# SDA us on pins AD1 and AD2

# Create an I2C device
i2c = busio.I2C(board.SCL, board.SDA)

# create a kit object
gimbal = ServoKit(channels=16, i2c=i2c)

pan_servo = 0 # channel 0

tilt_servo = 1 # channel 1

# test the servos
# gimbal.servo[pan_servo].angle = 0
# gimbal.servo[tilt_servo].angle = 0


# get the range of the servos
pan_servo_range = gimbal.servo[pan_servo].actuation_range
tilt_servo_range = gimbal.servo[tilt_servo].actuation_range

# print the range of the servos
print("Pan Servo Range: ", pan_servo_range)
print("Tilt Servo Range: ", tilt_servo_range)

# go to the neutral position
gimbal.servo[pan_servo].angle = 90
gimbal.servo[tilt_servo].angle = 90