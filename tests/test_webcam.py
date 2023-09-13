'''This script simple uses a webcam to capture images and display them on the screen.'''

# Import the OpenCV library
import cv2

# Create a VideoCapture object
cap = cv2.VideoCapture(0) # 0 is the default camera via USB

# set the width and height
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Check if the webcam is opened correctly
if not cap.isOpened():
    raise IOError("Cannot open webcam")

# loop until the user presses the q key
while True:
    # Read the current frame from the webcam
    ret, frame = cap.read()
    # Display the current frame in the window called "Webcam"
    # cv2.imshow("Webcam", frame)

    # resize the frame to be half the size
    frame = cv2.resize(frame, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)

    # display the frame
    cv2.imshow("Webcam", frame)

    # Wait for the user to press the q key
    c = cv2.waitKey(1)
    if c == ord('q'):
        break

# Release the VideoCapture object
cap.release()
# Destroy all windows
cv2.destroyAllWindows()

# End of program
