# -*- coding: utf-8 -*-
"""
Draws the lane lines on the test images.
"""

import pickle
import cv2
import numpy as np
import glob
from tracker import Tracker
import matplotlib.pyplot as plt
# import matplotlib.image as mpimg

# End of libraries

# Function definitions
def abs_sobel_thresh(img, orient='x', thresh=(0, 255)):
    # Apply the following steps to img
    # 1) Convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    # Apply x or y gradient with the OpenCV Sobel() function
    # and take the absolute value
    if orient == 'x':
        abs_sobel = np.absolute(cv2.Sobel(gray, cv2.CV_64F, 1, 0))
    if orient == 'y':
        abs_sobel = np.absolute(cv2.Sobel(gray, cv2.CV_64F, 0, 1))
    # Rescale back to 8 bit integer
    scaled_sobel = np.uint8(255*abs_sobel/np.max(abs_sobel))
    # Create a copy and apply the threshold
    binary_output = np.zeros_like(scaled_sobel)
    # Here I'm using inclusive (>=, <=) thresholds, but exclusive is ok too
    binary_output[(scaled_sobel >= thresh[0]) & (scaled_sobel <= thresh[1])] = 1
    # Return the result
    return binary_output

def mag_thresh(img, sobel_kernel=15, mag_thresh=(0, 255)):
    # Convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    # Take both Sobel x and y gradients
    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    # Calculate the gradient magnitude
    gradmag = np.sqrt(sobelx**2 + sobely**2)
    # Rescale to 8 bit
    scale_factor = np.max(gradmag)/255 
    gradmag = (gradmag/scale_factor).astype(np.uint8) 
    # Create a binary image of ones where threshold is met, zeros otherwise
    binary_output = np.zeros_like(gradmag)
    binary_output[(gradmag >= mag_thresh[0]) & (gradmag <= mag_thresh[1])] = 1

    # Return the binary image
    return binary_output

def dir_threshold(img, sobel_kernel=3, thresh=(0, np.pi/2)):
    # Grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    # Calculate the x and y gradients
    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    # Take the absolute value of the gradient direction, 
    # apply a threshold, and create a binary image result
    # absgraddir = np.arctan2(np.absolute(sobely), np.absolute(sobelx))
    with np.errstate(divide = 'ignore', invalid = 'ignore'):
        absgraddir = np.absolute(np.arctan(sobely / sobelx))
        binary_output =  np.zeros_like(absgraddir)
        # Apply threshold
        binary_output[(absgraddir >= thresh[0]) & (absgraddir <= thresh[1])] = 1
    # Return the binary image
    return binary_output

def color_threshold(img, sthresh=(0, 255), vthresh=(0, 255)):
    hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
    s_channel = hls[:,:,2]
    s_binary = np.zeros_like(s_channel)
    s_binary[ (s_channel >= sthresh[0]) & (s_channel <= sthresh[1]) ] = 1
    
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    v_channel = hsv[:,:,2]
    v_binary = np.zeros_like(v_channel)
    v_binary[ (v_channel >= vthresh[0]) & (v_channel <= vthresh[1]) ] = 1
    
    output = np.zeros_like(s_channel)
    output[(s_binary == 1) & (v_binary == 1)] = 1
    return output

def window_mask(width, height, img_ref, center, level):
    output = np.zeros_like(img_ref)
    output[int(img_ref.shape[0]-(level+1)*height):int(img_ref.shape[0] \
        - level*height), max(0, int(center - width)):min(int(center + width), \
            img_ref.shape[1])] = 1
    return output

# End of function definitions
    
    
# Program Main
    
# Read in the saved camera matrix and distortion coefficients
# These are the arrays calculated using cam_cal.py
dist_pickle = pickle.load( open( "calibration_pickle.p", "rb" ) )
mtx = dist_pickle["mtx"]
dist = dist_pickle["dist"]    
# Make a list of test images
images = glob.glob('./../CarND-Advanced-Lane-Lines/test_images/straight_lines*.jpg')

# Undistort the images in the list
for idx, fname in enumerate(images):
    img = cv2.imread(fname)
    img = cv2.undistort(img, mtx, dist, None, mtx)

    # Preprocess image and generate binary pixel of interest
    preprocessImage = np.zeros_like(img[:,:,0])
    gradx = abs_sobel_thresh(img, orient='x', thresh=(12, 255))
    grady = abs_sobel_thresh(img, orient='y', thresh=(25, 255))
    c_binary = color_threshold(img, sthresh=(100, 255), vthresh=(50, 255))
    preprocessImage[ ((gradx == 1) & (grady == 1) | (c_binary == 1)) ] = 255

    # Define the perspective transformation area
    img_size = (img.shape[1], img.shape[0])
    
    bot_width = 0.76  # percent of bottom trapezoid height
    mid_width = 0.08  # percent of middle trapezoid height
    height_pct = 0.62  # percent for trapezoid height
    bot_trim = 0.935  # percent from top to bottom to avoid car hood
    # Coordinates go clockwise for src and dst
    src = np.float32([[img.shape[1]*(0.5-mid_width/2), img.shape[0]*height_pct], \
                    [img.shape[1]*(0.5+mid_width/2), img.shape[0]*height_pct], \
                    [img.shape[1]*(0.5+bot_width/2), img.shape[0]*bot_trim], \
                    [img.shape[1]*(0.5-bot_width/2), img.shape[0]*bot_trim]])
    offset = img_size[0] * 0.25
    dst = np.float32([[offset, 0], [img_size[0]-offset, 0], \
                    [img_size[0]-offset, img_size[1]], [offset, img_size[1]]])
    """

    """
    # Perform the transorm
    M = cv2.getPerspectiveTransform(src, dst)
    Minv = cv2.getPerspectiveTransform(dst, src)
    # Warp the image using OpenCV warpPerspective()
    warped = cv2.warpPerspective(preprocessImage, M, img_size, flags=cv2.INTER_LINEAR)

    #  Drawing the polygon with the source points
    road = np.zeros_like(img)
    road_bkg = np.zeros_like(img)
    src_points = np.array([src], dtype=np.int32)
    dst_points = np.array([dst], dtype=np.int32)
    
    print(src_points)
    print(dst_points)
    cv2.fillPoly(road, np.array([dst], dtype=np.int32), color = [255, 0, 0])  #blue
    cv2.fillPoly(road, np.array([dst], dtype=np.int32), color = [0, 0, 255])  #red
    
    
    road_warped = cv2.warpPerspective(road, Minv, img_size, flags=cv2.INTER_LINEAR)
    road_warped_bkg = cv2.warpPerspective(road_bkg, Minv, img_size, flags=cv2.INTER_LINEAR)

    base = cv2.addWeighted(img, 1.0, road_warped_bkg, -1.0, 0.0)
    result = cv2.addWeighted(base, 1.0, road_warped, 1.0, 0.0)
    #result = cv2.warpPerspective(result, M, img_size, flags=cv2.INTER_LINEAR)

    
    plt.imshow(result)
    plt.show()
    
    # Write out the images
    write_name = './../CarND-Advanced-Lane-Lines/test_images/tracked' \
        + str(idx) + '.jpg'
    cv2.imwrite(write_name, result)
    