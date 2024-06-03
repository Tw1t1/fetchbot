# Copyright 2023 Tiziano Fiorenzani (modifications by Josh Newans)


import cv2
import numpy as np;


def find_circles(image, tuning_params):

    blur = 5

    #- Blur image to remove noise
    working_image = cv2.blur(image, (blur, blur)) 
    
    #- Convert image from BGR to HSV
    working_image = cv2.cvtColor(working_image, cv2.COLOR_BGR2HSV)    
       
    #- Apply HSV threshold
    thresh_min = (tuning_params["h_min"], tuning_params["s_min"], tuning_params["v_min"])
    thresh_max = (tuning_params["h_max"], tuning_params["s_max"], tuning_params["v_max"])
    working_image = cv2.inRange(working_image, thresh_min, thresh_max)

    
    # Dilate and Erode
    working_image = cv2.dilate(working_image, None, iterations=2)
    working_image = cv2.erode(working_image, None, iterations=2)

    
    # Make a copy of the image for tuning
    tuning_image = cv2.bitwise_and(image, image, mask = working_image)

    # Invert the image to suit the blob detector
    working_image = 255-working_image

    
    # Set up the SimpleBlobdetector with default parameters.
    params = cv2.SimpleBlobDetector_Params()
        
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

    detector = cv2.SimpleBlobDetector_create(params)
    
    # Run detection!
    keypoints = detector.detect(working_image)

    size_min_px = tuning_params['sz_min']*working_image.shape[1]/100.0
    size_max_px = tuning_params['sz_max']*working_image.shape[1]/100.0

    keypoints = [k for k in keypoints if k.size > size_min_px and k.size < size_max_px]

   
    # Set up main output image and tuning output image
    line_color=(0,0,255)

    out_image = cv2.drawKeypoints(image, keypoints, np.array([]), line_color, cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    tuning_image = cv2.drawKeypoints(tuning_image, keypoints, np.array([]), line_color, cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    
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
    return cv2.KeyPoint(x, y, kp.size/cv_image.shape[1])


def create_tuning_window(initial_values):
    cv2.namedWindow("Tuning", 0)
    cv2.createTrackbar("h_min","Tuning",initial_values['h_min'],180,no_op)
    cv2.createTrackbar("h_max","Tuning",initial_values['h_max'],180,no_op)
    cv2.createTrackbar("s_min","Tuning",initial_values['s_min'],255,no_op)
    cv2.createTrackbar("s_max","Tuning",initial_values['s_max'],255,no_op)
    cv2.createTrackbar("v_min","Tuning",initial_values['v_min'],255,no_op)
    cv2.createTrackbar("v_max","Tuning",initial_values['v_max'],255,no_op)
    cv2.createTrackbar("sz_min","Tuning",initial_values['sz_min'],100,no_op)
    cv2.createTrackbar("sz_max","Tuning",initial_values['sz_max'],100,no_op)


def get_tuning_params():
    trackbar_names = ["h_min","h_max","s_min","s_max","v_min","v_max","sz_min","sz_max"]
    return {key:cv2.getTrackbarPos(key, "Tuning") for key in trackbar_names}


def wait_on_gui():
    cv2.waitKey(2)


def no_op(x):
    pass



    """
    methodes not in use
    """
#---------- Draw search window: returns the image
# rect_px - window in adimensional units
def draw_window2(image, rect_px, color=(255,0,0), line=5,):
    #-- Draw a rectangle from top left to bottom right corner
    return cv2.rectangle(image,(rect_px[0],rect_px[1]),(rect_px[2],rect_px[3]),color,line)

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