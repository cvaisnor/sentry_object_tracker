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
                x_axis = joystick.get_axis(0)
                y_axis = joystick.get_axis(1)
                gimbal.set_velocity(x_axis, y_axis)
            elif event.type == pygame.JOYBUTTONDOWN:
                if joystick.get_button(0):
                    print("Moving to neutral position")
                    gimbal.move_to_neutral()
                elif joystick.get_button(1):
                    print("Stopping movement")
                    gimbal.set_velocity(0, 0)
        
        time.sleep(0.01)

except KeyboardInterrupt:
    print('Closing Program')
    gimbal.stop()
    pygame.quit()

# End of script