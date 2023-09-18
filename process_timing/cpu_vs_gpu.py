import cv2
import torch
import time
import numpy as np

# Load your images here
large_image_path = 'full.jpg'
small_image_path = 'crop.png'
large_image = cv2.imread(large_image_path, 0)
small_image = cv2.imread(small_image_path, 0)

# Convert to torch.Tensor
large_image_torch = torch.from_numpy(large_image).unsqueeze(0).unsqueeze(0).float()
small_image_torch = torch.from_numpy(small_image).unsqueeze(0).unsqueeze(0).float()

# Perform template matching with OpenCV
start = time.time()
res = cv2.matchTemplate(large_image, small_image, cv2.TM_CCORR_NORMED)
end = time.time()
opencv_time = end - start
min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
print(f"OpenCV (CPU):\nLocation: {max_loc}\nTime: {opencv_time:.6f} seconds")
print()

# Move tensor to GPU
if torch.cuda.is_available():
    large_image_torch = large_image_torch.cuda()
    small_image_torch = small_image_torch.cuda()

# Perform template matching with PyTorch
start = time.time()
res = torch.nn.functional.conv2d(large_image_torch, small_image_torch)
end = time.time()
pytorch_time = end - start
_, max_index = torch.max(res.view(res.shape[0], -1), dim=1)
y, x = np.unravel_index(max_index.cpu().numpy(), res.shape[-2:])
max_loc = (y, x)

print(f"PyTorch (GPU):\nLocation: ({x}, {y})\nTime: {pytorch_time:.6f} seconds")