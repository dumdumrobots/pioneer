import cv2
import matplotlib.pyplot as plt
import numpy as np

# To do: adaption to image size 28x28
 
cap = cv2.VideoCapture(0)
 
while(1):
 
    # Take each frame
    _, frame = cap.read()
 
    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
 
    # define range of red color in HSV
    useThresholdVar = 1
    # useThresholdVar = 2
    if useThresholdVar == 1:
        lower_yellow = np.array([20,100,100])
        upper_yellow = np.array([30,255,255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)  
    
    # if useThresholdVar == 2:
    #     # lower boundary RED color range values; Hue (0 - 10)
    #     lower1 = np.array([0, 100, 20])
    #     upper1 = np.array([10, 255, 255])
    #     # upper boundary RED color range values; Hue (160 - 180)
    #     lower2 = np.array([160,100,20])
    #     upper2 = np.array([179,255,255])
    #     #check if the HSV of the frame is lower or upper red
    #     lower_mask = cv2.inRange(frame, lower1, upper1)
    #     upper_mask = cv2.inRange(frame, lower2, upper2)
    #     mask = lower_mask + upper_mask
 
    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame,frame, mask= mask)
    
    # Draw rectangular bounded line on the detected red area
    #(ret, contours, hierarchy) = cv2.findContours(full_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    (contours, hierarchy) = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for pic,contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area > 50): #to remove the noise
            # Constructing the size of boxes to be drawn around the detected red area
            x,y,w,h = cv2.boundingRect(contour)
            frame = cv2.rectangle(frame, (x, y), (x+w, y+h), (255,0,0), 2)
 
     
    cv2.imshow('Result',frame)
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break
 
cv2.destroyAllWindows()
cap.release()