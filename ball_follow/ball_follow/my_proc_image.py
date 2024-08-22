# Copyright 2023 Tiziano Fiorenzani (modifications by Yosef Seada)


import cv2
import numpy as np

prev_image = None

def find_circles(image, tuning_params, tuning_mode):
    """
    Detect circles (balls) in the given image using color thresholding and contour analysis.
    
    Args:
        image (np.array): Input image in BGR format
        tuning_params (dict): Dictionary of parameters for tuning the detection
    
    Returns:
        tuple: (normalized keypoints, output image, tuning image)
    """
    global prev_img
    # Extract parameters
    thresh_min = (tuning_params["h_min"], tuning_params["s_min"], tuning_params["v_min"])
    thresh_max = (tuning_params["h_max"], tuning_params["s_max"], tuning_params["v_max"])
    search_window = [tuning_params["x_min"], tuning_params["y_min"], tuning_params["x_max"], tuning_params["y_max"]]

    search_window_px = convert_rect_perc_to_pixels(search_window, image)

    if prev_img is None:
        prev_img = image
    alpha = 0.2

    beta = (1.0 - alpha)

    image = cv2.addWeighted(image, alpha, prev_img, beta, 0.0)

    # Image preprocessing
    blurred = cv2.GaussianBlur(image, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, thresh_min, thresh_max)
    

    # Noise reduction
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3,3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3,3), np.uint8))


    # Apply the search window
    working_image = apply_search_window(mask, search_window)

    contours, _ = cv2.findContours(working_image.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # contours = imutils.grab_contours(contours)

    keypoints = []

    for c in contours:
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        
        # if M["m00"] != 0:
        #     center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        # else: 
        #     center = (int(x), int(y))

        kp = cv2.KeyPoint(x, y, radius*2)
        keypoints.append(kp)
            
    size_min_px = tuning_params['sz_min']*working_image.shape[1]/100.0
    size_max_px = tuning_params['sz_max']*working_image.shape[1]/100.0

    keypoints = [k for k in keypoints if k.size > size_min_px and k.size < size_max_px]

    out_image = None
    tuning_image = None
    if tuning_mode:
        out_image = image.copy()
        tuning_image = cv2.bitwise_and(image, image, mask=mask)

        line_color = (0,0,255)
        # Set up main output image
        out_image = cv2.drawKeypoints(out_image, keypoints, np.array([]), line_color, cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        out_image = draw_window2(out_image, search_window_px)


        # Set up tuning output image
        tuning_image = cv2.drawKeypoints(tuning_image, keypoints, np.array([]), line_color, cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        tuning_image = draw_window2(tuning_image, search_window_px)
 
    keypoints_normalised = [normalise_keypoint(working_image, k) for k in keypoints]
    
    return keypoints_normalised, out_image, tuning_image


#Apply search window: returns the image
def apply_search_window(image, window_adim=[0.0, 0.0, 1.0, 1.0]):
    rows = image.shape[0]
    cols = image.shape[1]
    x_min_px    = int(cols*window_adim[0]/100)
    y_min_px    = int(rows*window_adim[1]/100)
    x_max_px    = int(cols*window_adim[2]/100)
    y_max_px    = int(rows*window_adim[3]/100)    
    
    # Initialize the mask as a black image
    mask = np.zeros(image.shape,np.uint8)
    
    # Copy the pixels from the original image corresponding to the window
    mask[y_min_px:y_max_px,x_min_px:x_max_px] = image[y_min_px:y_max_px,x_min_px:x_max_px]   
    
    return (mask)


# Draw search window: returns the image
def draw_window2(image,              #- Input image
                rect_px,             #- window in adimensional units
                color=(255,0,0),    
                line=5,             
               ):
    
    # Draw a rectangle from top left to bottom right corner
    return cv2.rectangle(image, (rect_px[0], rect_px[1]), (rect_px[2], rect_px[3]), color, line)


def convert_rect_perc_to_pixels(rect_perc, image):
    rows = image.shape[0]
    cols = image.shape[1]

    scale = [cols, rows, cols, rows]

    return [int(a*b/100) for a,b in zip(rect_perc, scale)]


def normalise_keypoint(cv_image, kp):
    
    height, width = cv_image.shape[:2]
    
    center_x = 0.5*width
    center_y = 0.5*height
    
    x = (kp.pt[0] - center_x)/(center_x)
    y = (kp.pt[1] - center_y)/(center_y)
    
    return cv2.KeyPoint(x, y, kp.size/cv_image.shape[1])


def create_tuning_window(initial_values):
    cv2.namedWindow("Tuning", 0)
    no_op = lambda x : None
    cv2.createTrackbar("x_min", "Tuning", initial_values['x_min'], 100, no_op)
    cv2.createTrackbar("x_max", "Tuning", initial_values['x_max'], 100, no_op)
    cv2.createTrackbar("y_min", "Tuning", initial_values['y_min'], 100, no_op)
    cv2.createTrackbar("y_max", "Tuning", initial_values['y_max'], 100, no_op)
    cv2.createTrackbar("h_min", "Tuning", initial_values['h_min'], 180, no_op)
    cv2.createTrackbar("h_max", "Tuning", initial_values['h_max'], 180, no_op)
    cv2.createTrackbar("s_min", "Tuning", initial_values['s_min'], 255, no_op)
    cv2.createTrackbar("s_max", "Tuning", initial_values['s_max'], 255, no_op)
    cv2.createTrackbar("v_min", "Tuning", initial_values['v_min'], 255, no_op)
    cv2.createTrackbar("v_max", "Tuning", initial_values['v_max'], 255, no_op)
    cv2.createTrackbar("sz_min", "Tuning", initial_values['sz_min'], 100, no_op)
    cv2.createTrackbar("sz_max", "Tuning", initial_values['sz_max'], 100, no_op)
    


def get_tuning_params():
    trackbar_names = [
        "x_min","x_max","y_min","y_max",
        "h_min","h_max","s_min","s_max","v_min","v_max",
        "sz_min","sz_max"]
    return {key:cv2.getTrackbarPos(key, "Tuning") for key in trackbar_names}


def wait_on_gui():
    cv2.waitKey(2)