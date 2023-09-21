# import the necessary packages
import cv2

from camera_functions import capture_single_frame, get_contours

def nothing(x):
    pass


def main():
    '''Main function.'''

    # Create a VideoCapture object
    camera_capture = cv2.VideoCapture(0) # 0 is the default camera via USB

    # # set the width and height
    camera_capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    camera_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    background_frame = capture_single_frame(camera_capture)

    print('-'*30)
    print('Sentry Camera Contour Tuning')
    print('-'*30)

    # create a small window for the trackbars
    cv2.namedWindow('Trackbars', cv2.WINDOW_NORMAL)

    # create trackbars for the parameters
    cv2.createTrackbar('CONTOUR_THRESHOLD_VALUE', 'Trackbars', 40, 255, nothing)
    cv2.createTrackbar('MIN_AREA', 'Trackbars', 10, 100, nothing)
    cv2.createTrackbar('MAX_AREA', 'Trackbars', 300, 1000, nothing)


    while True:

        # get the current positions of the trackbars
        CONTOUR_THRESHOLD_VALUE = cv2.getTrackbarPos('CONTOUR_THRESHOLD_VALUE', 'Trackbars')
        MIN_AREA = cv2.getTrackbarPos('MIN_AREA', 'Trackbars')
        MAX_AREA = cv2.getTrackbarPos('MAX_AREA', 'Trackbars')

        # capture frame
        current_frame = capture_single_frame(camera_capture)

        contours, threshold_frame, difference_frame = get_contours(background_frame, current_frame, threshold_value=CONTOUR_THRESHOLD_VALUE)

        # on the current frame, draw a rectangle at the center of the frame based on the min area
        cv2.rectangle(current_frame, (int(current_frame.shape[1]/2 - MIN_AREA/2), int(current_frame.shape[0]/2 - MIN_AREA/2)), (int(current_frame.shape[1]/2 + MIN_AREA/2), int(current_frame.shape[0]/2 + MIN_AREA/2)), (0, 255, 0), 2)

        # draw a second rectangle at the center of the frame based on the max area
        cv2.rectangle(current_frame, (int(current_frame.shape[1]/2 - MAX_AREA/2), int(current_frame.shape[0]/2 - MAX_AREA/2)), (int(current_frame.shape[1]/2 + MAX_AREA/2), int(current_frame.shape[0]/2 + MAX_AREA/2)), (0, 0, 255), 2)

        # display the frames
        cv2.imshow('Current Frame', current_frame)
        cv2.imshow("Threshold Frame", threshold_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            # cleanup
            camera_capture.release()
            cv2.destroyAllWindows()
            break

if __name__ == "__main__":
    main()