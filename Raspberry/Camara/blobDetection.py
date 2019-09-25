#!/usr/bin/python

# Standard imports
import cv2
import argparse
import imutils
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", help = "path to the image")
ap.add_argument("-c", "--color", help = "color that needs detection")
args = vars(ap.parse_args())

color = args["color"]

image = cv2.imread(args["image"])
image = cv2.resize(image, (600,600))
hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# define the list of boundaries
boundaries = [
    ([94, 80, 20], [130, 255, 255], False, "b"),   #Blue
    ([80, 50, 20], [100, 255, 255], True, "r"),    #Red (Since red in the bounds for hsv, we invert and look for cyan)
    ([32, 100, 20], [75, 255, 255], False, "g")    #Green
]
 
# loop over the boundaries
for (lower, upper, invert, col) in boundaries:
    if col == color :
        # create NumPy arrays from the boundaries
        lower = np.array(lower, dtype = "uint8")
        upper = np.array(upper, dtype = "uint8")

        to_use = hsv_image
        if invert:
            inverted = (255-image)
            to_use = cv2.cvtColor(inverted, cv2.COLOR_BGR2HSV)
    
        # find the colors within the specified boundaries and apply
        # the mask
        mask = cv2.inRange(to_use, lower, upper)
        output = cv2.bitwise_and(to_use, to_use, mask = mask)
 
# Read image
#im = cv2.imread("pista_bloque2.jpeg", cv2.IMREAD_GRAYSCALE)

im = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(im, (5, 5), 0)
thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]

# find contours in the thresholded image
cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
	cv2.CHAIN_APPROX_SIMPLE)
cnts = imutils.grab_contours(cnts)

# loop over the contours
for c in cnts:
	# compute the center of the contour
	M = cv2.moments(c)
	cX = int(M["m10"] / M["m00"])
	cY = int(M["m01"] / M["m00"])
 
	# draw the contour and center of the shape on the image
	cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
	cv2.circle(image, (cX, cY), 7, (255, 255, 255), -1)
	cv2.putText(image, "center", (cX - 20, cY - 20),
		cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
 
	# show the image
	cv2.imshow("Image", image)
	cv2.waitKey(0)
'''
# Setup SimpleBlobDetector parameters.
params = cv2.SimpleBlobDetector_Params()

# Change thresholds
params.minThreshold = 10
params.maxThreshold = 200


# Filter by Area.
params.filterByArea = True
params.minArea = 0

# Filter by Circularity
params.filterByCircularity = True
params.minCircularity = 0.0

# Filter by Convexity
params.filterByConvexity = True
params.minConvexity = 0.0
    
# Filter by Inertia
params.filterByInertia = True
params.minInertiaRatio = 0.00

# Create a detector with the parameters
ver = (cv2.__version__).split('.')
if int(ver[0]) < 3 :
	detector = cv2.SimpleBlobDetector(params)
else : 
	detector = cv2.SimpleBlobDetector_create(params)


# Detect blobs.
keypoints = detector.detect(im)

# Draw detected blobs as red circles.
# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures
# the size of the circle corresponds to the size of blob

im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
# Show blobs
cv2.imshow("Keypoints", im_with_keypoints)
'''

#cv2.imshow("Keypoints", thresh)
#cv2.waitKey(0)