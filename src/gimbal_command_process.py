'''This script holds the gimbal class for controlling the gimbal using Visca over IP on a separate process'''
import multiprocessing as mp
from visca_over_ip import Camera


class Gimbal(mp.Process):
    '''This class is used to control the gimbal using Visca over IP on a separate process.'''
    
    def __init__(self, ip_address):
        self.ip_address = ip_address
        self.command_queue = mp.Queue(1) # initialize the command queue
        self.response_queue = mp.Queue(1)
        super().__init__(target=self._target)
        self.start()

    def _target(self):
        gimbal = Camera(self.ip_address, port=52381) # default port for Visca over IP
        gimbal.pantilt_home()

        # start the command queue loop
        while True:
            command = self.command_queue.get()
            if command == 'home':
                gimbal.pantilt_home()
                self.response_queue.put(True)
            elif command == 'close':
                gimbal.close_connection()
                self.response_queue.put(True)
                break
            else:
                pan_speed, tilt_speed = command
                gimbal.pantilt(pan_speed, tilt_speed)

    def go_to_home(self):
        '''Go to the home position.'''
        # check if the queue is empty
        if not self.command_queue.empty():
            self.command_queue.get()
        self.command_queue.put('home')
        self.response_queue.get()

    def move(self, pan_speed, tilt_speed):
        '''Move the gimbal.'''
        # check if the queue is empty
        if not self.command_queue.empty():
            self.command_queue.get()
        self.command_queue.put((pan_speed, tilt_speed))
    
    def close(self):
        '''Close the gimbal.'''
        # check if the queue is empty
        if not self.command_queue.empty():
            self.command_queue.get()
        self.command_queue.put('close')
        self.response_queue.get()
        super().close()

