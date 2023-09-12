'''This script holds the class for the identified object.'''

import cv2
import numpy as np

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