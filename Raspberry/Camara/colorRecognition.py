import numpy as np
import argparse
import cv2
 
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", help = "path to the image")
args = vars(ap.parse_args())
 
image = cv2.imread(args["image"])
image = cv2.resize(image, (600,600))
hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# define the list of boundaries
boundaries = [
    ([94, 80, 20], [130, 255, 255], False),   #Blue
    ([80, 50, 20], [100, 255, 255], True),    #Red (Since red in the bounds for hsv, we invert and look for cyan)
    ([32, 100, 20], [75, 255, 255], False)    #Green
]

# loop over the boundaries
for (lower, upper, invert) in boundaries:
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
 
    # show the images
    cv2.imshow("images", np.hstack([image, output]))
    cv2.waitKey(0)