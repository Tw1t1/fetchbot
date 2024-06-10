# Copyright 2023 Tiziano Fiorenzani (modifications by Josh Newans)


import cv2 as cv
import numpy as np;
import imutils # to insatll: $ pip install --upgrade imutils


def find_circles(image, tuning_params):
    thresh_min = (tuning_params["h_min"], tuning_params["s_min"], tuning_params["v_min"])
    thresh_max = (tuning_params["h_max"], tuning_params["s_max"], tuning_params["v_max"])
    
    # a = find_circles_a(image, thresh_min, thresh_max) 
    # b = find_circles_b(image, thresh_min, thresh_max) 
    c = find_circles_c(image, tuning_params)
    
    return c




def find_circles_a(image, thresh_min, thresh_max):
    
    center = None
    radius = None

    blurred = cv.GaussianBlur(image, (5, 5), 0)
    hsv = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)
	
	# construct a mask for the color "green", then perform
	# a series of dilations and erosions to remove any small blobs left in the mask
    mask = cv.inRange(hsv, thresh_min, thresh_max)
    mask = cv.erode(mask, None, iterations=2)
    mask = cv.dilate(mask, None, iterations=2)
	
    # find contours in the mask and initialize the current (x, y) center of the ball
    cnts = cv.findContours(mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    
    # only proceed if at least one contour was found
    if len(cnts) > 0:

        detect_bal = True
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and centroid
        c = max(cnts, key=cv.contourArea)
        ((x, y), radius) = cv.minEnclosingCircle(c)
        M = cv.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # contourIdx represent index of contours, -1 mean all contours 
        cv.drawContours(image, cnts, contourIdx=-1, color=(128,0,128), thickness=2)

    return center, radius

def find_circles_b(image, thresh_min, thresh_max):
    
    radius = -1
    center = (-1, -1)

    # Convert BGR to HSV
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)

    # Create masks for the target color range
    mask = cv.inRange(hsv, thresh_min, thresh_max)

    # Apply morphological operations to clean up the mask
    kernel = np.ones((5,5), np.uint8)

    mask = cv.dilate(mask, kernel, iterations=1)

    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel, 2)
    mask = cv.GaussianBlur(mask, (5, 5), 0)
    

    circles = cv.HoughCircles(mask, cv.HOUGH_GRADIENT, 1.2, 120, 120, 50, 10, 0)

    if circles is not None and circles[0][0].ndim == 1:
        # Convert the (x, y, r) values to integers
        circles = np.uint16(np.around(circles))

        for i in circles[0,:]:

            
            if i[2] < 5: # ignore objects with radius small then 5 pixels
                continue

            if i[2] > radius:

                detect_bal = True
                center = (i[0], i[1])
                radius = i[2]
                
        
        # detect_bal = True
        # chosen = None
        # for c in circles[0, :]:
        #     if chosen is None: chosen = c
        #     if prev_circle is not None:
        #         if dist(chosen[0], chosen[1], prev_circle[0], prev_circle[1]) <= dist(c[0], c[1], prev_circle[0], prev_circle[1]):
        #             chosen = c
        #     center = (chosen[0], chosen[1])
        #     radius = chosen[2]
        #     distance = calculate_distance(radius*2)

    return detect_bal, center, radius

def find_circles_c(image, tuning_params):
    blur = 5
    
    #- Blur image to remove noise
    working_image = cv.blur(image, (blur, blur)) 
    
    #- Convert image from BGR to HSV
    working_image = cv.cvtColor(working_image, cv.COLOR_BGR2HSV)    
       
    #- Apply HSV threshold
    thresh_min = (tuning_params["h_min"], tuning_params["s_min"], tuning_params["v_min"])
    thresh_max = (tuning_params["h_max"], tuning_params["s_max"], tuning_params["v_max"])
    working_image = cv.inRange(working_image, thresh_min, thresh_max)

    
    # Dilate and Erode
    working_image = cv.dilate(working_image, None, iterations=2)
    working_image = cv.erode(working_image, None, iterations=2)

    
    # Make a copy of the image for tuning
    tuning_image = cv.bitwise_and(image, image, mask = working_image)

    # Invert the image to suit the blob detector
    working_image = 255-working_image

    
    # Set up the SimpleBlobdetector with default parameters.
    params = cv.SimpleBlobDetector_Params()
        
    # Change thresholds
    params.minThreshold = 0
    params.maxThreshold = 100
    
    # Filter by Area.
    params.filterByArea = True
    params.minArea = 30
    params.maxArea = 20000
        
    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.1
        
    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.5
    
        
    # Filter by Inertia
    params.filterByInertia =True
    params.minInertiaRatio = 0.5

    detector = cv.SimpleBlobDetector_create(params)
    
    # Run detection!
    keypoints = detector.detect(working_image)

    size_min_px = tuning_params['sz_min']*working_image.shape[1]/100.0
    size_max_px = tuning_params['sz_max']*working_image.shape[1]/100.0

    keypoints = [k for k in keypoints if k.size > size_min_px and k.size < size_max_px]

   
    # Set up main output image and tuning output image
    line_color=(0,0,255)

    out_image = cv.drawKeypoints(image, keypoints, np.array([]), line_color, cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    tuning_image = cv.drawKeypoints(tuning_image, keypoints, np.array([]), line_color, cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    
    keypoints_normalised = [normalise_keypoint(working_image, k) for k in keypoints]

    return keypoints_normalised, out_image, tuning_image


def normalise_keypoint(cv_image, kp):
    
    rows = float(cv_image.shape[0])
    cols = float(cv_image.shape[1])
    # print(rows, cols)
    center_x    = 0.5*cols
    center_y    = 0.5*rows
    # print(center_x)
    x = (kp.pt[0] - center_x)/(center_x)
    y = (kp.pt[1] - center_y)/(center_y)
    return cv.KeyPoint(x, y, kp.size/cv_image.shape[1])



def create_tuning_window(initial_values):
    cv.namedWindow("Tuning", 0)
    cv.createTrackbar("h_min","Tuning",initial_values['h_min'],180,no_op)
    cv.createTrackbar("h_max","Tuning",initial_values['h_max'],180,no_op)
    cv.createTrackbar("s_min","Tuning",initial_values['s_min'],255,no_op)
    cv.createTrackbar("s_max","Tuning",initial_values['s_max'],255,no_op)
    cv.createTrackbar("v_min","Tuning",initial_values['v_min'],255,no_op)
    cv.createTrackbar("v_max","Tuning",initial_values['v_max'],255,no_op)
    cv.createTrackbar("sz_min","Tuning",initial_values['sz_min'],100,no_op)
    cv.createTrackbar("sz_max","Tuning",initial_values['sz_max'],100,no_op)


def get_tuning_params():
    trackbar_names = ["h_min","h_max","s_min","s_max","v_min","v_max","sz_min","sz_max"]
    return {key:cv.getTrackbarPos(key, "Tuning") for key in trackbar_names}


def wait_on_gui():
    cv.waitKey(2)


def no_op(x):
    pass



    """
    methodes not in use
    """
#---------- Draw search window: returns the image
# rect_px - window in adimensional units
def draw_window2(image, rect_px, color=(255,0,0), line=5,):
    #-- Draw a rectangle from top left to bottom right corner
    return cv.rectangle(image,(rect_px[0],rect_px[1]),(rect_px[2],rect_px[3]),color,line)

#---------- Apply search window: returns the image
def apply_search_window(image, window_adim=[0.0, 0.0, 100.0, 100.0]):
    rows = image.shape[0]
    cols = image.shape[1]
    x_min_px = int(cols*window_adim[0]/100)
    y_min_px = int(rows*window_adim[1]/100)
    x_max_px = int(cols*window_adim[2]/100)
    y_max_px = int(rows*window_adim[3]/100)    
    
    #--- Initialize the mask as a black image
    mask = np.zeros(image.shape,np.uint8)
    
    #--- Copy the pixels from the original image corresponding to the window
    mask[y_min_px:y_max_px,x_min_px:x_max_px] = image[y_min_px:y_max_px,x_min_px:x_max_px]   
    
    #--- return the mask
    return(mask)


def convert_rect_perc_to_pixels(rect_perc, image):
    rows = image.shape[0]
    cols = image.shape[1]

    scale = [cols, rows, cols, rows]

    return [int(a*b/100) for a,b in zip(rect_perc, scale)]