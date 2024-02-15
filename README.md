# Sentry Camera Object Tracker

Project Contents:
- archive: old code
- images: images for README.md
- src: main functions
- stepper_serial_control: Arduino code for stepper motor control
- tests: for camera, motion detection, Yolov8 object identification

Needed if using FT232H/PCA9685 controller:
```bash
export BLINKA_FT232H='1'
```

Temp fix for "global cap_v4l.cpp:1119 tryIoctl VIDEOIO(V4L2:/dev/video0): select() timeout" error:
```bash
sudo rmmod uvcvideo
sudo modprobe uvcvideo nodrop=1 timeout=5000
```
(possible other fix):
```bash
export OPENCV_VIDEOIO_PRIORITY_MSMF=0
```

**v1**:

<img src="images/v1.jpg" width="50%" height="50%">

**v2**:

<img src="images/v2.jpg" width="50%" height="50%">

**v2.2**:

<img src="images/v2_2.jpeg" width="50%" height="50%">

---
