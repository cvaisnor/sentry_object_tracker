# Sentry Camera Object Tracker

**Web Interface**:

<img src="images/flask_app.png" width="85%" height="85%">

## To run Flask app:
```bash
python app.py
```
- **Web interface is @ localhost:5000**

**Linux Bug**:
Temp fix for "global cap_v4l.cpp:1119 tryIoctl VIDEOIO(V4L2:/dev/video0): select() timeout" error:
```bash
sudo rmmod uvcvideo && sudo modprobe uvcvideo nodrop=1 timeout=5000
```
- This needs to be ran every time before a script is ran that uses the camera.

**Version 4**:

<img src="images/v4.jpeg" width="65%" height="65%">

**Version 3**:

<img src="images/v3.jpeg" width="60%" height="60%">

**Version 2.2**:

<img src="images/v2_2.jpeg" width="60%" height="60%">

**Version 2**:

<img src="images/v2.jpg" width="60%" height="60%">

**Version 1 (Servo Motors)**:

<img src="images/v1.jpg" width="60%" height="60%">