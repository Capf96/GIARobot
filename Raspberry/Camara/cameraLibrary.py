# Standard imports
import cv2
import argparse
import imutils
import numpy as np

def boat_center(color) :

    # Condition
    center = False
    # Initialize video stream
    cap = cv2.VideoCapture(0)
    # define the list of boundaries
    boundaries = [
    ([94, 80, 20], [130, 255, 255], False, "b"),   #Blue
    ([80, 50, 20], [100, 255, 255], True, "r"),    #Red (Since red in the bounds for hsv, we invert and look for cyan)
    ([32, 100, 20], [75, 255, 255], False, "g")    #Green
    ]

    while(not center) :
        # Capture frame from video
        ret, frame = cap.read()
        cv2.imshow('frame',frame)
        if cv2.waitKey(20) & 0xFF == ord('q'):
            break

        # Prepare image
        image = cv2.resize(frame.copy(), (600,600))
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
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
                im = cv2.bitwise_and(to_use, to_use, mask = mask)

        im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(im, (5, 5), 0)
        thresh = cv2.threshold(blurred, 100, 255, cv2.THRESH_BINARY)[1]

        # find contours in the thresholded image
        cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        c = max(cnts, key=len, default=[[]])
        if len(c)<100 :
            continue

        # compute the center of the contour cX = int(M["m10"] / M["m00"])
        M = cv2.moments(c)
        try:
            cX = int(M["m10"] / M["m00"])
        except :
            continue

        if(250<=cX<=350) : 
            center = True
    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

