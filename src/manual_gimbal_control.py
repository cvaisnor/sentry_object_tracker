# script for manual gimbal control with a joystick, no camera

import time
import pygame
from gimbal_control_system import GimbalController

# Initialize pygame
pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Initialize gimbal controller
gimbal = GimbalController()

gimbal.run_homing()

# Main loop
print('Starting manual gimbal control...')
print('-' * 30)

try:
    while True:
        # Get joystick events
        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                x_axis = joystick.get_axis(0) # range from -1 to 1
                y_axis = joystick.get_axis(1) # range from -1 to 1
                
                # scale ranges to -1000 to 1000 and round to integers
                x_axis = int(x_axis * 1000)
                y_axis = int(y_axis * 1000)


                # only send velocity command if joystick is outside of deadzone -100 to 100
                if x_axis > 100 or x_axis < -100 or y_axis > 100 or y_axis < -100:
                    # print(f'x: {x_axis}, y: {y_axis}')
                    gimbal.set_velocity(x_axis, y_axis)
            elif event.type == pygame.JOYBUTTONDOWN:
                if joystick.get_button(0):
                    print("Moving to neutral position")
                    # gimbal.move_to_neutral()
                elif joystick.get_button(1):
                    print("Stopping movement")
                    # gimbal.set_velocity(0, 0)
        
        time.sleep(0.1)

except KeyboardInterrupt:
    print('Closing Program')
    # gimbal.close()
    pygame.quit()

# End of script