import Adafruit_GPIO.FT232H as FT232H
from adafruit_servokit import ServoKit

# Create an FT232H object
FT232H.use_FT232H()

# Create an I2C device at address 0x40 on FT232H
i2c = FT232H.I2CDevice(0x40)

gimbal = ServoKit(channels=16, i2c=i2c)  # using PCA9685 with I2C of FT232H

pan_servo = 0 # channel 0
pan_servo_range = [0, 180]

tilt_servo = 1 # channel 1
tilt_servo_range = [90, 160]

# test the servos
gimbal.servo[pan_servo].angle = 60
gimbal.servo[tilt_servo].angle = 120