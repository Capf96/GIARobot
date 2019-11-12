#!/usr/bin/python

# Standard imports
import cv2
import argparse
import imutils
import numpy as np

# Parse arguments 
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", help = "path to the image")
ap.add_argument("-c", "--color", help = "color that needs detection")
args = vars(ap.parse_args())

# Process arguments
color = args["color"]
#image = cv2.imread(args["image"])
cap = cv2.VideoCapture(0)
ret, frame = cap.read()
image = frame.copy()
# Prepare image
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

im = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(im, (5, 5), 0)
thresh = cv2.threshold(blurred, 100, 255, cv2.THRESH_BINARY)[1]

# find contours in the thresholded image
cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
cnts = imutils.grab_contours(cnts)
# If cnts is not empty then there is a contour
c = max(cnts, key=len, default=[[]])

# draw the contour and center of the shape on the image
cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
# compute the center of the contour cX = int(M["m10"] / M["m00"])
M = cv2.moments(c)
cX = int(M["m10"] / M["m00"])

if(250<=cX<=350) : 
    print(True)
else :
    print(False)

# show the image
cv2.imshow("Image", image)
cv2.waitKey(0)