'''This script tests each servo.'''

from adafruit_servokit import ServoKit # Import the ServoKit library
import time # Import the time library

# Create an object called my_kit to access the ServoKit library
my_kit = ServoKit(channels=16) # using PCA9685

# Define the servos
pan_servo = 0 # channel 0
tilt_servo = 1 # channel 1

# test set tilt to 0 then 180 then 0
my_kit.servo[tilt_servo].angle = 100
time.sleep(1)
my_kit.servo[tilt_servo].angle = 180
time.sleep(1)
my_kit.servo[tilt_servo].angle = 100