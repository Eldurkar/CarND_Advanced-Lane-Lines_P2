# -*- coding: utf-8 -*-
"""
Created on Sun Feb  9 08:21:50 2020

@author: eldurkar
"""


import numpy as np
import cv2

class Tracker():
    
    def __init__(self, Mywindow_width, Mywindow_height, Mymargin, My_ym = 1, \
                 My_xm = 1, Mysmooth_factor = 15):
        # List that stores all the pas (left, right) center values used for
        # smoothing the output.
        self.recent_centers = []
        # Window pixel width of the center values, used to count pixels inside
        # center windows to determine curve values.
        self.window_width = Mywindow_width
        # Window pixel height of the center values, used to count pixels inside
        # center windows to determine curve values. 
        # Breaks image into vertical levels.
        self.window_height = Mywindow_height
        # The pixel distance in both directions to slide (left window 
        # + right window) template for searching.
        self.margin = Mymargin
        # Meters per pixel in vertical axis.
        self.ym_per_pix = My_ym
        # Meters per pixel in horizontal axis.
        self.xm_per_pix = My_xm
        # Averages over the centers and smoothens the result
        self.smooth_factor = Mysmooth_factor
        
    # The main tracking function for finding and storing lane segment positions.
    def find_window_centroids(self, warped):
        window_width = self.window_width
        window_height = self.window_height
        margin = self.margin
        # Store the (left, right) window centroid positions per level.
        window_centroids = []
        # Create a window template that will be used for convolution.
        window = np.ones(window_width)
        
        # First find the starting positions for the left and right lane by
        # using np.sum to get the vertical image slice and then np.convolve  
        # the vertical image slice with the window template.
        
        # Sum bottom quarter of the image to get slice (ratio can be different)
        l_sum = np.sum(warped[int(3*warped.shape[0]/4):, :int(warped.shape[1]/2)], \
                       axis = 0)
        l_center = np.argmax(np.convolve(window, l_sum)) - window_width/2
        r_sum = np.sum(warped[int(3*warped.shape[0]/4):, int(warped.shape[1]/2):], \
                       axis = 0)
        r_center = np.argmax(np.convolve(window, r_sum)) - window_width/2 \
            + int(warped.shape[1]/2)
            
        # Add what was found for the first layer
        window_centroids.append((l_center, r_center))
        
        # Go through each leayerlooking for max pixel locations
        for level in range(1, (int)(warped.shape[0]/window_height)):
            # Convovle the window into the vertical slice of the image
            image_layer = np.sum(warped[int(warped.shape[0]-(level+1) \
                        * window_height):int(warped.shape[0]-level \
                        * window_height),:], axis = 0)
            conv_signal = np.convolve(window, image_layer)
            # Find the best left centroid by using past left center as reference.
            # Use window_width/2 as offset because the convolution signal is at
            # the right side of the window, not at the center of window.
            offset = window_width/2
            l_min_index = int(max(l_center + offset - margin, 0))
            l_max_index = int(min(l_center + offset + margin, warped.shape[1]))
            l_center = np.argmax(conv_signal[l_min_index:l_max_index]) \
                + l_min_index - offset
            # Find the best right centroid by using past right center as reference.
            r_min_index = int(max(r_center + offset - margin, 0))
            r_max_index = int(min(r_center + offset + margin, warped.shape[1]))
            r_center = np.argmax(conv_signal[r_min_index:r_max_index]) \
                + r_min_index - offset
            # Add what was found for that layer.
            window_centroids.append((l_center, r_center))
            
        self.recent_centers.append(window_centroids)
        # Return averaged values of the line centers, helps prevent the markers
        # from jumping around.
        return np.average(self.recent_centers[-self.smooth_factor:], axis = 0)