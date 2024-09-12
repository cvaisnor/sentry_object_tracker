# Sentry Camera Object Tracker

Instructions:
- In main.py, change line #26 for using the VISCA command protocol. 
    - SERIAL = True # Serial connection via arduino
    - SERIAL = False # VISCA PTZ Camera

Project Contents:
- archive: old code
- images: images for README.md
- src: main functions
- stepper_serial_control: Arduino code for stepper motor control
- tests: for camera, motion detection, Yolov8 object identification

**Linux Bug**:
Temp fix for "global cap_v4l.cpp:1119 tryIoctl VIDEOIO(V4L2:/dev/video0): select() timeout" error:
```bash
sudo rmmod uvcvideo && sudo modprobe uvcvideo nodrop=1 timeout=5000
```

**Version 4**:

<img src="images/final_serial_version.jpeg" width="75%" height="75%">


**Version 3**:

<img src="images/v3.jpeg" width="60%" height="60%">

**Version 2.2**:

<img src="images/v2_2.jpeg" width="60%" height="60%">

**Version 2**:

<img src="images/v2.jpg" width="60%" height="60%">

**Version (Servo Motors) 1**:

<img src="images/v1.jpg" width="60%" height="60%">