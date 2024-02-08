### Object Following, detection through webcam stream 
import numpy as np
import cv2
import imutils
from imutils.video import VideoStream
import time

#RED SPHERICAL OBJECTS HSV RANGES
Lowerbound = (0,100,100)
Upperbound = (10,255,255)

'''
#Yellow
Lowerbound = (29, 86, 6)
Upperbound = (64, 255, 255)

#BLUE
Lowerbound = (100,100,100)
Upperbound = (140,255,255)

#GREEN
Lowerbound = (40,40,40)
Upperbound = (80,255,255)
'''
cap = VideoStream(src=0).start() #webcam video stream
time.sleep(2.0)

while True:
	frame = cap.read() #read frame by frame
    
    #image prefiltering
	frame = imutils.resize(frame, width=500) #resizing
	blurred = cv2.GaussianBlur(frame, (5, 5), 0) #blurring
	hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV) #change from RGB to HSV
	
	#Noise removal
	thresh = cv2.inRange(hsv, Lowerbound, Upperbound) #thresholding
	mask = cv2.erode(thresh, None, iterations=2) #erosion removes white noise but also shrinks the object
	mask = cv2.dilate(mask, None, iterations=2) #dilating the eroded img now won't bring back the noise, but object area increases
	
	countours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	countours = imutils.grab_contours(countours)
	centroid = None
	
	if len(countours) > 0:
		c = max(countours, key=cv2.contourArea) #largest countour
		((x, y), radius) = cv2.minEnclosingCircle(c) #minimum enclosing circle for the largest countour
		
		M = cv2.moments(c) 
		''' 
		Dictionary returned from cv2.moments()
		m00: The zeroth moment, which is the area of the object.
        m10: The first moment about the x-axis, which is the x-coordinate of the center of mass.
        m01: The first moment about the y-axis, which is the y-coordinate of the center of mass.
        m20: The second moment about the x-axis, which is the variance of the object's distribution around the x-axis.
        m02: The second moment about the y-axis, which is the variance of the object's distribution around the y-axis.
        m11: The covariance of the object's distribution around the x- and y-axes.
		'''
		centroid = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])) #centroid of detected object
		print(centroid)
	    
		cv2.circle(frame, (int(x), int(y)), int(radius),
				(0, 0, 255), 2)
		cv2.circle(frame, centroid, 5, (0, 255, 255), -1)
	
	cv2.imshow("Frame", frame)
	cv2.imshow("Grayscale-prefiltered", thresh)
	if cv2.waitKey(1) & 0xFF == ord('q'): #exit if q is pressed
		break

cap.stop()
# close all windows
cv2.destroyAllWindows()
