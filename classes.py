'''This script holds the classes for the sentry gimbal project.'''

class Servo:
    
    def init(self, servo, servo_range):
        self.servo = servo
        self.servo_range = servo_range
        self.neutral = int(servo_range / 2)
        self.position = self.neutral # inital position is neutral
    
    def set_neutral(self):
        self.servo.position = self.neutral
    
    def move_to_position(self, position):
        if position in self.servo_range:
            self.servo.position = position
        else:
            print(f'Position {position} is not in range {self.servo_range} of servo {self.servo}')
    
    def move_by_degrees(self, degrees):
        # get the current position
        position = self.servo.angle

        # calculate the new position
        new_position = position + degrees

        if new_position in self.servo_range:
            self.servo.angle = new_position
        else:
            print(f'Position {new_position} is not in range {self.servo_range} of servo {self.servo}')


class Static_Camera:
    
    def init(self, camera_capture_width, camera_capture_height):
        self.camera_capture_width = camera_capture_width
        self.camera_capture_height = camera_capture_height
        self.center_x = int(camera_capture_width / 2)
        self.center_y = int(camera_capture_height / 2)
        self.center = (self.center_x, self.center_y)


class IdentifiedObject:

    def __init__(self, x, y, w, h):
        self.x = x
        self.y = y
        self.width = w
        self.height = h
        self.center = self.get_center(x, y, w, h)
    
    def get_center(self, x, y, w, h):
        '''Returns the center of the identified object.'''
        center_x = int(x + w / 2)
        center_y = int(y + h / 2)
        return center_x, center_y
    

class Gimbal:
    
    def __init__(self, pan_servo, tilt_servo):
        self.pan_servo = pan_servo
        self.tilt_servo = tilt_servo
    
    def set_neutral(self):
        for servo in self.servo:
            servo.set_neutral()
    
    def move_tilt_up(self, degrees):
        self.tilt_servo.move_by_degrees(degrees)
    
    def move_tilt_down(self, degrees):
        self.tilt_servo.move_by_degrees(-degrees)

    def move_pan_left(self, degrees):
        self.pan_servo.move_by_degrees(degrees)

    def move_pan_right(self, degrees):
        self.pan_servo.move_by_degrees(-degrees)

