# Sentry Camera Object Tracker

Project Contents:
- archive: old code
- images: images for README.md and PyTorch timing test
- src: main functions (main_steppers.py)
- stepper_serial_control: Arduino code for stepper motor control
- tests: for webcam, servo, motion detection, FT232H/PCA9685 control boards, cpu vs gpu comparison

TODO:
- Measure the time difference between check_image_match and check_image_match_local
- Work on a PyTorch version of check_image_match
- allow speed and steps to be configured by the distance of the object from the center of the frame
- attach shorting pins to enable smaller steps
- requirements.txt

Needed if using FT232H/PCA9685 controller:
```bash
export BLINKA_FT232H='1'
```
Screen View:

<img src="images/sentry_object_tracker.png" width="50%" height="50%">

**v1**:

<img src="images/servo_setup.jpg" width="50%" height="50%">

**v2**:

<img src="images/v2.jpg" width="50%" height="50%">

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