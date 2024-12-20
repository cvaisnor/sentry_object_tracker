import cv2
import math

def create_trackbar_window():
    # create a window for the trackbars
    cv2.namedWindow('Tracking Parameters')

    # create trackbars for the parameters
    cv2.createTrackbar('CONTOUR_THRESHOLD_VALUE', 'Tracking Parameters', 240, 255, lambda x: None) # default 40
    cv2.createTrackbar('MIN_AREA', 'Tracking Parameters', 100, 400, lambda x: None) # default 100
    cv2.createTrackbar('MAX_AREA', 'Tracking Parameters', 500, 1000, lambda x: None) # default 500
    cv2.createTrackbar('TEMPLATE_MATCHING_THRESHOLD', 'Tracking Parameters', 70, 98, lambda x: None) # default 70
    cv2.createTrackbar('PIXEL_BUFFER', 'Tracking Parameters', 10, 50, lambda x: None) # default 10
    cv2.createTrackbar('FRAMES_TO_AVERAGE', 'Tracking Parameters', 1, 10, lambda x: None) # default 1
    cv2.createTrackbar('GIMBAL MOVEMENT', 'Tracking Parameters', 0, 1, lambda x: None)


def read_trackbar_values():
    # get the current positions of the trackbars
    contour_threshold_value = cv2.getTrackbarPos('CONTOUR_THRESHOLD_VALUE', 'Tracking Parameters')
    min_area = cv2.getTrackbarPos('MIN_AREA', 'Tracking Parameters')
    max_area = cv2.getTrackbarPos('MAX_AREA', 'Tracking Parameters')
    template_matching_threshold = cv2.getTrackbarPos('TEMPLATE_MATCHING_THRESHOLD', 'Tracking Parameters') / 100
    pixel_buffer = cv2.getTrackbarPos('PIXEL_BUFFER', 'Tracking Parameters')
    frames_to_average = cv2.getTrackbarPos('FRAMES_TO_AVERAGE', 'Tracking Parameters')
    gimbal_movement = cv2.getTrackbarPos('GIMBAL MOVEMENT', 'Tracking Parameters')

    return contour_threshold_value, min_area, max_area, template_matching_threshold, pixel_buffer, frames_to_average, gimbal_movement


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


def check_image_match_local(full_image_to_search, cropped_object_image, last_loc=(0,0), obj_padding=100, match_method=cv2.TM_CCOEFF_NORMED):
    """
    Look for small_image in full_image and return best and worst results
    For More Info See
    http://docs.opencv.org/3.1.0/d4/dc6/tutorial_py_template_matching.html
    """
    # Define the x and y starting and ending coordinates of the ROI
    obj_size = cropped_object_image.shape
    
    y_start = max(0, last_loc[1] - obj_padding)
    y_end = min(full_image_to_search.shape[0], last_loc[1] + obj_size[0] + obj_padding)
    x_start = max(0, last_loc[0] - obj_padding)
    x_end = min(full_image_to_search.shape[1], last_loc[0] + obj_size[1] + obj_padding)

    # Extract ROI from the image
    roi = full_image_to_search[y_start:y_end, x_start:x_end]
    
    # Perform template matching on the ROI
    result = cv2.matchTemplate(roi, cropped_object_image, match_method)

    # Find the maximum correlation location
    _, max_val, _, max_loc = cv2.minMaxLoc(result)
    
    # Convert max_loc to coordinates relating to the full image
    max_loc = (max_loc[0] + x_start, max_loc[1] + y_start)

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

    # lambda function to eliminate contours that are zero
    contours = list(filter(lambda x: cv2.contourArea(x) > 1, contours))
    # print('Area of contours:', [cv2.contourArea(x) for x in contours])
    
    return contours, thresh_image, difference


def get_largest_contour(contours):
    '''Returns the largest contour.'''
    largest_contour = None
    largest_contour_area = 0
    for contour in contours:
        if cv2.contourArea(contour) > largest_contour_area:
            largest_contour = contour
            largest_contour_area = cv2.contourArea(contour)
            # print('Largest contour area:', largest_contour_area)
    return largest_contour


def combine_contours(contours, min_area=100, max_area=10000):
    '''Returns the min and max x and y values for the contours.'''
    min_x, min_y, max_x, max_y = 10000, 10000, 0, 0   
    for contour in contours:
        if cv2.contourArea(contour) < min_area or cv2.contourArea(contour) > max_area:
            continue

        x, y, w, h = cv2.boundingRect(contour)
        
        # update min and max x and y based on contour
        min_x = min(x, min_x)
        min_y = min(y, min_y)
        max_x = max(x + w, max_x)
        max_y = max(y + h, max_y)

    return min_x, min_y, max_x, max_y


def contour_parser(contours, min_area, max_area, frame_width, frame_height):

    valid_countours = []

    contour_found = False
    x, y, w, h = 0, 0, 0, 0

    if len(contours) > 0:
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)

            bounding_box_area = w * h
            # check contour density by comparing the area of the contour to the area of the bounding box
            contour_density = cv2.contourArea(contour) / (bounding_box_area)

            if contour_density < 0.8:
                continue

            # check minimum and maximum area of contour based on the area of the bounding box
            if bounding_box_area < min_area or bounding_box_area > max_area:
                continue

            # check if it is too close to the edge of the frame
            if x + w > frame_width or y + h > frame_height:
                continue

            valid_countours.append(contour)

    if len(valid_countours) > 0:
        largest_countour = get_largest_contour(valid_countours) # low computational cost, no sorting required

        # get the x, y, width, and height of box around the contour
        x, y, w, h = cv2.boundingRect(largest_countour)

        contour_found = True

    return contour_found, x, y, w, h


def detect_objects_yolo(frame, model):
    '''
    Input: frame 
    Output: three lists - boxes, confidences, class_ids
    '''
    results = model(frame, stream=True)
    boxes = []
    confidences = []
    class_ids = []

    for r in results:
        for box in r.boxes:
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            boxes.append([x1, y1, x2, y2])
            confidence = math.ceil((box.conf[0]*100))/100
            confidences.append(confidence)
            cls = int(box.cls[0])
            class_ids.append(cls)
    
    return boxes, confidences, class_ids