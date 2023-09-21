import cv2
import torch
import numpy as np

def capture_single_frame(camera_capture):
    '''Captures a single frame from the camera.'''
    ret, frame = camera_capture.read()
    if frame is None:
        print("Frame is empty")
        return False
    return frame


def get_cropped_object_image(frame, x, y, w, h):
    '''Returns the cropped image.'''
    cropped_object_image = frame[y:y+h, x:x+w]
    return cropped_object_image


def check_image_match(full_image_to_search, cropped_object_image, match_method=cv2.TM_CCOEFF_NORMED):
    """
    Look for small_image in full_image and return best and worst results
    For More Info See
    http://docs.opencv.org/3.1.0/d4/dc6/tutorial_py_template_matching.html
    """
    # returns a grayscale image, where each pixel denotes how much does the neighbourhood of that pixel match with template

    result = cv2.matchTemplate(full_image_to_search, cropped_object_image, match_method)

    # Process result data and return probability values and
    # xy Location of best and worst image match
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
    return max_val, max_loc


def check_image_match_local(full_image_to_search, cropped_object_image, max_loc, match_method=cv2.TM_CCOEFF_NORMED):
    """
    Look for small_image in full_image and return best and worst results
    For More Info See
    http://docs.opencv.org/3.1.0/d4/dc6/tutorial_py_template_matching.html
    """
    # returns a grayscale image, where each pixel denotes how much does the neighbourhood of that pixel match with template

    # TODO: get this working
    # limit the search area to a smaller area around the previous match, to speed up the search
    # x = max_loc[0]
    # y = max_loc[1]
    # w = cropped_object_image.shape[1]
    # h = cropped_object_image.shape[0]

    # object_center_x = x + w / 2
    # object_center_y = y + h / 2

    # # to find the next match
    # y_lower = int(object_center_y - 150)
    # y_upper = int(object_center_y + 150)
    # x_lower = int(object_center_x - 150)
    # x_upper = int(object_center_x + 150)

    # # ensure the search area is still valid
    # if y_lower < 0:
    #     y_lower = 0
    # if y_upper > full_image_to_search.shape[0]:
    #     y_upper = full_image_to_search.shape[0]
    # if x_lower < 0:
    #     x_lower = 0
    # if x_upper > full_image_to_search.shape[1]:
    #     x_upper = full_image_to_search.shape[1]

    # full_image_to_search = full_image_to_search[y_lower:y_upper, x_lower:x_upper]


    # print('full_image_to_search.shape: ', full_image_to_search.shape)
    # print('cropped_object_image.shape: ', cropped_object_image.shape)

    result = cv2.matchTemplate(full_image_to_search, cropped_object_image, match_method)

    # Process result data and return probability values and
    # xy Location of best and worst image match
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
    return max_val, max_loc


def get_contours(old_frame, current_frame, threshold_value=40.0):
    '''Returns the contours, threshold frame, and difference frame.'''
    old_frame_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)
    current_frame_gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
    
    old_frame_blurred = cv2.GaussianBlur(old_frame_gray, (21, 21), 0)
    current_frame_blurred = cv2.GaussianBlur(current_frame_gray, (21, 21), 0)
    
    difference = cv2.absdiff(old_frame_blurred, current_frame_blurred)
    
    ret, thresh_image = cv2.threshold(difference, threshold_value, 255, cv2.THRESH_BINARY)
    thresh_image = cv2.dilate(thresh_image, None, iterations=2)

    contours, _ = cv2.findContours(thresh_image.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    return contours, thresh_image, difference


def get_largest_contour(contours, min_area=100, max_area=10000):
    '''Returns the largest contour.'''
    largest_contour = None
    largest_contour_area = 0
    for contour in contours:
        if cv2.contourArea(contour) < min_area or cv2.contourArea(contour) > max_area:
            continue

        if cv2.contourArea(contour) > largest_contour_area:
            largest_contour = contour
            largest_contour_area = cv2.contourArea(contour)
    return largest_contour


def combine_contours(contours, user_param_min_area, user_param_max_area):
    '''Returns the min and max x and y values for the contours.'''
    min_x, min_y, max_x, max_y = 10000, 10000, 0, 0   
    for contour in contours:
        if cv2.contourArea(contour) < 100 or cv2.contourArea(contour) > 1000:
            continue

        x, y, w, h = cv2.boundingRect(contour)
        
        # update min and max x and y based on contour
        min_x = min(x, min_x)
        min_y = min(y, min_y)
        max_x = max(x + w, max_x)
        max_y = max(y + h, max_y)

    return min_x, min_y, max_x, max_y


def template_matching_pytorch(full_image, object_to_find):
    """
    Perform template matching with PyTorch.
    :param full_image: the image to search
    :param object_to_find: the object to find
    :return: the location of the object
    """
    # create tensor from openCV images
    # full image is [480, 640, 3]
    # object to find is [100, 100, 3]

    # convert to grayscale
    full_image = cv2.cvtColor(full_image, cv2.COLOR_BGR2GRAY)
    object_to_find = cv2.cvtColor(object_to_find, cv2.COLOR_BGR2GRAY)

    print('full_image.shape: ', full_image.shape)
    print('object_to_find.shape: ', object_to_find.shape)

    full_image = torch.from_numpy(full_image).unsqueeze(0).unsqueeze(0).float()
    object_to_find = torch.from_numpy(object_to_find).unsqueeze(0).unsqueeze(0).float()

    # Move tensor to GPU
    if torch.cuda.is_available():
        full_image = full_image.cuda()
        object_to_find = object_to_find.cuda()

    # Perform template matching with PyTorch and return the location of the object, and a similarity score
    # conv2d expects a 4D tensor, so we need to unsqueeze the first two dimensions

    res = torch.nn.functional.conv2d(full_image, object_to_find)
    _, max_index = torch.max(res.view(res.shape[0], -1), dim=1)
    y, x = np.unravel_index(max_index.cpu().numpy(), res.shape[-2:])
    max_loc = (y, x)

    # release GPU memory
    # del full_image
    # del object_to_find
    # del res
    # del max_index

    return max_loc