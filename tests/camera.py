'''This script simple uses a webcam to capture images and display them on the screen.'''

# Import the OpenCV library
import cv2
import time

# Create a VideoCapture object
cap = cv2.VideoCapture(0) # 0 is the default camera via USB

print("Default resolution: " + str(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) + "x" + str(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
print("Default FPS: " + str(cap.get(cv2.CAP_PROP_FPS)))
print("Default fourcc: " + str(cap.get(cv2.CAP_PROP_FOURCC)))
print("Default buffersize: " + str(cap.get(cv2.CAP_PROP_BUFFERSIZE)))

# set the width and height
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
cap.set(cv2.CAP_PROP_FPS, 60)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
print()

print("Default resolution: " + str(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) + "x" + str(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
print("Default FPS: " + str(cap.get(cv2.CAP_PROP_FPS)))
print("Default fourcc: " + str(cap.get(cv2.CAP_PROP_FOURCC)))
print("Default buffersize: " + str(cap.get(cv2.CAP_PROP_BUFFERSIZE)))

time.sleep(1)

# Check if the webcam is opened correctly
if not cap.isOpened():
    raise IOError("Cannot open webcam")

# loop until the user presses the q key
loop_counter = 0
try:
    while True:
        start = time.time()

        # Read the current frame from the webcam
        ret, frame = cap.read()
        if ret == False:
            print("No frame captured, closing program")
            cap.release()
            cv2.destroyAllWindows()
            break
        
        loop_counter += 1
        
        end = time.time()

        cap_read_time = end-start

        # # display the loop counter in the top left corner of the frame
        # cv2.putText(frame, str(loop_counter), (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        # cv2.putText(frame, str(cap_read_time), (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

        # Display the captured frame
        cv2.imshow("Webcam", frame)

        # Wait for the user to press the q key
        c = cv2.waitKey(1)
        if c == ord('q'):
            print('Closing program')
            cap.release()
            cv2.destroyAllWindows()
            break

except KeyboardInterrupt:
    print('Closing program')
    cap.release()
    cv2.destroyAllWindows()

# Release the VideoCapture object
cap.release()
cv2.destroyAllWindows()
