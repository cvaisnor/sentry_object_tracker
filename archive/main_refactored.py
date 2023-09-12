'''This script uses a pan and tilt servo to control a gimbal with a webcam attached. It uses OpenCV to detect motion and moves the servos to track the motion.'''

import cv2
import numpy as np
from adafruit_servokit import ServoKit
import time


class ServoTracker:
    def __init__(self, channels):
        self.gimbal = ServoKit(channels=channels)
        self.static_background = None
        self.last_motion_time = 0
        self.frame_width = 640
        self.frame_height = 480

        self.config = {
            "pan_servo": {
                "channel": 0,
                "range": [0, 180],
            },
            "tilt_servo": {
                "channel": 1,
                "range": [90, 160],
            },
            "camera": {
                "channel": 0
            }
        }
        self.video = cv2.VideoCapture(self.config['camera']['channel'])
        self.set_neutral()

    def set_neutral(self):
        for serv in ['pan_servo', 'tilt_servo']:
            neutral = int(sum(self.config[serv]["range"]) / 2)
            self.gimbal.servo[self.config[serv]["channel"]].angle = neutral

    def move_to_position(self, serv, position):
        serv_range = self.config[serv]["range"]
        if position in range(serv_range[0], serv_range[1]):
            self.gimbal.servo[self.config[serv]["channel"]].angle = position
        else:
            print(f"Position out of range\nServo: {serv}\nPosition: {position}")
    
    def get_servo_position(self, serv):
        return self.gimbal.servo[self.config[serv]["channel"]].angle

    def run(self):
        ret, frame = self.video.read()
        if frame is None:
            print("Frame is empty")
            return False


        gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred_img = cv2.GaussianBlur(gray_img, (21, 21), 0)

        if time.time() - self.last_motion_time > 1:
            self.static_background = blurred_img
            self.last_motion_time = time.time()

        diff_frame = cv2.absdiff(self.static_background, blurred_img)

        threshold_frame = cv2.threshold(diff_frame, 30, 255, cv2.THRESH_BINARY)[1]
        threshold_frame = cv2.dilate(threshold_frame, None, iterations = 2)

        contours, _ = cv2.findContours(threshold_frame.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            min_x, min_y, max_x, max_y = 10000, 10000, 0, 0
            
            for contour in contours:
                if cv2.contourArea(contour) < 500:
                    continue

                x, y, w, h = cv2.boundingRect(contour)
                # update min and max x and y based on contour
                min_x = min(x, min_x)
                min_y = min(y, min_y)
                max_x = max(x + w, max_x)
                max_y = max(y + h, max_y)

            cv2.rectangle(frame, (min_x, min_y), (max_x, max_y), (0, 255, 0), 3)

            cv2.circle(frame, (int(min_x + (max_x - min_x) / 2), int(min_y + (max_y - min_y) / 2)), 5, (0, 0, 255), -1)

            center_of_rectangle = (int(min_x + (max_x - min_x) / 2), int(min_y + (max_y - min_y) / 2))

            # get the color of the center of the rectangle
            center_color = frame[center_of_rectangle[1], center_of_rectangle[0]]

            # get the HSV values of the center of the rectangle
            hsv_center_color = cv2.cvtColor(np.uint8([[center_color]]), cv2.COLOR_BGR2HSV)

            # track the color of the center of the rectangle for the next 5 seconds
            if time.time() - self.last_motion_time < 5:
                # define range of color in HSV
                lower_color = np.array([hsv_center_color[0][0][0] - 10, 100, 100])
                upper_color = np.array([hsv_center_color[0][0][0] + 10, 255, 255])

                # Threshold the HSV image to get only the center color
                mask = cv2.inRange(frame, lower_color, upper_color)

                # Bitwise-AND mask and original image
                res = cv2.bitwise_and(frame, frame, mask=mask)

                # convert to grayscale
                res = cv2.cvtColor(res, cv2.CO

            center_x = center_of_rectangle[0]
            center_y = center_of_rectangle[1]

            pan_error = center_x - self.frame_width / 2
            tilt_error = center_y - self.frame_height / 2

            # get current servo positions
            pan_pos = self.get_servo_position('pan_servo')
            tilt_pos = self.get_servo_position('tilt_servo')

            # update servo positions
            if pan_error > 0:
                pan_pos += 1
            else:
                pan_pos -= 1

            if tilt_error > 0:
                tilt_pos -= 1
            else:
                tilt_pos += 1

            # move servos to new positions
            self.move_to_position('pan_servo', pan_pos)
            self.move_to_position('tilt_servo', tilt_pos)


        if cv2.waitKey(1) == ord('q'):
            return False

        # show the frame
        cv2.imshow("Frame", frame)

        # show threshold frame
        cv2.imshow("Threshold Frame", threshold_frame)

        # show original frame
        cv2.imshow("Original Frame", self.static_background)

        return True


def main():
    tracker = ServoTracker(channels=16)

    cv2.namedWindow("Servo Control")
    cv2.createTrackbar("pan", "Servo Control", 90, 180, lambda x: None)
    cv2.createTrackbar("tilt", "Servo Control", 125, 180, lambda x: None)

    continue_running = True
    while continue_running:
        continue_running = tracker.run()
        
    
    tracker.video.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
