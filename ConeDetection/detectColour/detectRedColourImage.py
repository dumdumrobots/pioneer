import cv2
import matplotlib.pyplot as plt
import numpy as np
from os import listdir
from os.path import isfile, join

# To do: adaption to image size 28x28
 
folder_path = "queryCones"
output_path = "output"

files = [ f for f in listdir(folder_path) if isfile(join(folder_path,f)) ]
images = np.empty(len(files), dtype=object)

for n in range(0, len(files)):
    frame = cv2.imread( join(folder_path,files[n]) )

    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # define range of red color in HSV
    useThresholdVar = 3
    # useThresholdVar = 2
    if useThresholdVar == 1:
        lower_red = np.array([160,50,50])
        upper_red = np.array([180,255,255])
        mask = cv2.inRange(hsv, lower_red, upper_red)  
        
    if useThresholdVar == 2:
        # lower boundary RED color range values; Hue (0 - 10)
        lower1 = np.array([5, 50, 80])
        upper1 = np.array([15, 100, 100])
        # upper boundary RED color range values; Hue (160 - 180)
        lower2 = np.array([160,50,20])
        upper2 = np.array([180,255,255])
        #check if the HSV of the frame is lower or upper red
        lower_mask = cv2.inRange(frame, lower1, upper1)
        upper_mask = cv2.inRange(frame, lower2, upper2)
        mask = lower_mask + upper_mask
    
    if useThresholdVar == 3: #detect orange
        lower_red = np.array([0,125,225])
        upper_red = np.array([7,240,255])
        mask = cv2.inRange(hsv, lower_red, upper_red)
    
    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame,frame, mask= mask)
        
    # Draw rectangular bounded line on the detected red area
    #(ret, contours, hierarchy) = cv2.findContours(full_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    (contours, hierarchy) = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for pic,contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area > 500): #to remove the noise
            # Constructing the size of boxes to be drawn around the detected red area
            x,y,w,h = cv2.boundingRect(contour)
            frame = cv2.rectangle(frame, (x, y), (x+w, y+h), (255,0,0), 2)
            
    cv2.imwrite(join(output_path,files[n]), frame)

# frameResized = cv2.resize(frame, (960, 540))       
# cv2.imshow('Result',frameResized)
# cv2.waitKey(0) 
# cv2.destroyAllWindows()
# cap.release()