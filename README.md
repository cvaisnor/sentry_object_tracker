# Sentry Camera Object Tracker

Project Contents:
- archive: old code
- process_timing: timing tests for various image processing methods
- tests: for webcam, servo, motion detection, and FT232H/PCA9685 controllers
- configs: tuning scripts for motion detection

TODO:
- Add functionality for stepper motor gimbal with Arduino
- requirements.txt

Needed if using FT232H/PCA9685 controller:
```bash
export BLINKA_FT232H='1'
```
Screen View:

<img src="images/sentry_object_tracker.png" width="50%" height="50%">

Current Servo Gimbal Setup:

<img src="images/servo_setup.jpg" width="50%" height="50%">

---
## Template Matching Speed Test

**AMD Ryzen 9 7950X 16/32 vs. Nvidia 4090**

Full: 6000 x 4000 pixels

Crop: 151 x 144 pixels

```
python process_timing/cpu_vs_gpu.py
```

OUTPUT:
```
OpenCV (CPU):
Location: (3180, 3174)
Time: 0.495360 seconds

PyTorch (GPU):
Location: ([3808], [987])
Time: 0.037448 seconds
```