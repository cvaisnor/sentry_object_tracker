# sentry_object_tracker


Project Contents:
- archive: old code
- tests: for webcam, servo, motion detection, and FT232H/PCA9685 controllers


For ubuntu-box:
```bash

export BLINKA_FT232H='1'
```
---
## Template matching speed test

**AMD Ryzen 9 7950X 16/32 vs. Nvidia 4090**

Full: 6000 x 4000 pixels
Crop: 151 x 144 pixels

```
python template_matching_images/cpu_vs_gpu.py
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