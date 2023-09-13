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
pan_servo_range = [0, 180]

tilt_servo = 1 # channel 1
tilt_servo_range = [90, 160]

# test the servos
gimbal.servo[pan_servo].angle = 60
gimbal.servo[tilt_servo].angle = 120

