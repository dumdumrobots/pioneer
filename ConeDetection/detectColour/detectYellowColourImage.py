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

#store the detected locations of red:
# x_array = []
# y_array = []
# w_array = []
# h_array = []

for n in range(0, len(files)):
    frame = cv2.imread( join(folder_path,files[n]) )

    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # define range of red color in HSV
    # useThresholdVar = 1
    useThresholdVar = 2
    if useThresholdVar == 1:
        lower_yellow = np.array([20,100,100])
        upper_yellow = np.array([30,255,255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)  
        
    if useThresholdVar == 2:
        lower_yellow = np.array([20,190,210])
        upper_yellow = np.array([30,255,255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)  
        
    
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
            #add the rectangle to the list
            # x_array.append(x)
            # y_array.append(y)
            # w_array.append(w)
            # h_array.append(h)
            frame = cv2.rectangle(frame, (x, y), (x+w, y+h), (255,0,0), 2)
            
    cv2.imwrite(join(output_path,files[n]), frame)

# frameResized = cv2.resize(frame, (960, 540))       
# cv2.imshow('Result',frameResized)
# cv2.waitKey(0) 
# cv2.destroyAllWindows()
# cap.release()