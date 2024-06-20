# Sentry Camera Object Tracker

## Branches:
### main: Uses the VISCA commands to control the gimbal (for PTZ camera w/ the VISCA protocol)
### serial: Uses serial commands for the custom stepper motor gimbal

Project Contents:
- archive: old code
- images: images for README.md
- src: main functions
- stepper_serial_control: Arduino code for stepper motor control
- tests: for camera, motion detection, Yolov8 object identification


**Linux Bug**:
Temp fix for "global cap_v4l.cpp:1119 tryIoctl VIDEOIO(V4L2:/dev/video0): select() timeout" error:
```bash
sudo rmmod uvcvideo && sudo modprobe uvcvideo nodrop=1 timeout=5000 && python main.py
```

**3D Printed Serial Version**:

<img src="images/final_serial_version.jpeg" width="90%" height="90%">

- for older versions, see /images folder
