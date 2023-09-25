import cv2

from archive.gopro_webcam import GoProWebcamPlayer


# https://github.com/gopro/OpenGoPro/blob/main/demos/python/multi_webcam/multi_webcam/webcam.py
gopro = GoProWebcamPlayer(serial='646')

gopro.open()

# https://gopro.github.io/OpenGoPro/python_sdk/api.html#open_gopro.api.params.WebcamFOV

gopro.webcam.start(gopro.port, resolution=12, fov=0)

gopro.player.url = GoProWebcamPlayer.STREAM_URL.format(port=gopro.port)

video_capture = cv2.VideoCapture(gopro.player.url + "?overrun_nonfatal=1&fifo_size=50000000", cv2.CAP_FFMPEG)

while True:
    # Capture frame-by-frame
    ret, frame = video_capture.read()

    # Display the resulting frame
    cv2.imshow('GoPro Webcam', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything is done, release the capture
video_capture.release()

gopro.close()

cv2.destroyAllWindows()
