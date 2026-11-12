#!/usr/bin/env python

import cv2
import numpy as np

# ========================= Student's code starts here =========================

# Params for camera calibration
x1 = 234
y1 = 100
x2 = 228
y2 = 165

theta = - np.arctan((234-228)/(65))
beta = 64/.1
tx = 230
ty = 91

# Function that converts image coord to world coord
def IMG2W(col, row):
    x = 0
    y = 0
    coords = np.array([col, row, 0])
    coords[0] = coords[0] - tx
    coords[1] = coords[1] - ty
    coords = coords * beta
    coords[2] = .03
    R = np.array([
        [np.cos(theta), np.sin(theta), 0], 
        [-np.sin(theta), np.cos(theta), 0], 
        [0,0,1]]).T

    coords = R @ (coords)

    return coords

    

# ========================= Student's code ends here ===========================

def blob_search(image_raw, color):

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # ========================= Student's code starts here =========================

    # Filter by Color
    params.filterByColor = False

    # Filter by Area.
    params.filterByArea = True
    params.minArea = 0
    params.maxArea = 1000

    # Filter by Circularity
    params.filterByCircularity = False
    #params.minCircularity = 0.0
    #params.maxCircularity = 0.5

    # Filter by Inerita
    params.filterByInertia = False
    #params.minInertiaRatio = 0.5
    #params.maxInertiaRatio = 1
    # Filter by Convexity
    params.filterByConvexity = False

    # ========================= Student's code ends here ===========================

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)

    # ========================= Student's code starts here =========================

    #calibration tool
    #lower = (0, 180, 200)
    #upper = (14, 220, 240)
    #
    
    #pink
    lower = (165, 160, 100)
    upper = (180, 255, 255)
    
    #green
    lower2 = (51, 100, 70)
    upper2 = (71, 255, 255)
    # Define a mask using the lower and upper bounds of the target color
    if (color == "pink"):
        mask_image = cv2.inRange(hsv_image, lower, upper)
    else:
        mask_image = cv2.inRange(hsv_image, lower2, upper2)

    # ========================= Student's code ends here ===========================

    keypoints = detector.detect(mask_image)

    # Find blob centers in the image coordinates
    blob_image_center = []
    num_blobs = len(keypoints)
    for i in range(num_blobs):
        blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))

    # ========================= Student's code starts here =========================

    # Draw the keypoints on the detected block
    im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints, None)

    # ========================= Student's code ends here ===========================

    xw_yw = []

    if(num_blobs == 0):
        print("No block found!")
    else:
        # Convert image coordinates to global world coordinate using IM2W() function
        for i in range(num_blobs):
            xw_yw.append(IMG2W(blob_image_center[i][0], blob_image_center[i][1]))


    cv2.namedWindow("Camera View")
    cv2.imshow("Camera View", image_raw)
    cv2.namedWindow("Mask View")
    cv2.imshow("Mask View", mask_image)
    cv2.namedWindow("Keypoint View")
    cv2.imshow("Keypoint View", im_with_keypoints)

    cv2.waitKey(2)

    return xw_yw
