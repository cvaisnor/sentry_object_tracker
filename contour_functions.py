import cv2

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