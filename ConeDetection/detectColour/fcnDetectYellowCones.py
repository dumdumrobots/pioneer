import cv2
import matplotlib.pyplot as plt
import numpy as np
from os import listdir
from os.path import isfile, join

def detectYellowCones(frame,image_divide_yellow,lower_yellow,upper_yellow):

    #initalize the arrays to store the detected objects
    array_yellow_centerXYwh = []
    array_yellow_4points = []
    
    #for the other colours:
    #avoid detecting one object as two different objects,
    #start with the object detection that are most reliable
    array_detectedAll_4points = []
    array_detectedAll_centerXYwh = []      
  
    image_height = frame.shape[0] 
    image_width = frame.shape[1]  
    boxMinArea = image_height*image_width/image_divide_yellow #adapt this: how near should the cone be s.t. we want to detect it
    MinArea = boxMinArea/100
        
    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    #for colour detection: create the mask_yellow
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)    
    # Bitwise-AND mask_yellow and original image
    res = cv2.bitwise_and(frame,frame, mask= mask_yellow)

    #check how big the area is
    (contours, hierarchy) = cv2.findContours(mask_yellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for pic,contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area > MinArea): #to remove the noise
            # Constructing the size of boxes to be drawn around the detected yellow area
            xBottomDown,yBottomDown,w,h = cv2.boundingRect(contour)
            x = xBottomDown+w/2
            y = yBottomDown+h/2
            if(w*h>boxMinArea):
                coneAlreadyDetected = False
                for kk in range(0,len(array_yellow_centerXYwh)):
                    XYwh_existing = array_yellow_centerXYwh[kk]
                    x_existing = XYwh_existing[0]
                    y_existing = XYwh_existing[1]
                    w_existing = XYwh_existing[2]
                    h_existing = XYwh_existing[3]
                    #the yellow cone is separated by a green area
                    #this is why there are two yellow cones detected for each yellow cone
                    #-->stop that by checking if the two yellow areas are below of each other
                    if x >= x_existing-w_existing/2 and x <= x_existing+w_existing/2:
                        #cone already detected
                        coneAlreadyDetected = True
                    #check if the cone lies. that means that there are two yellow areas detected near each other
                    if y > y_existing -h_existing/2 and y <= y_existing +h_existing/2 and x+2*w >= x_existing-w_existing/2 and x+2*w <= x_existing+w_existing/2:
                        coneAlreadyDetected = True
                                   
                if coneAlreadyDetected == True:
                    #cone already detected
                    frame = cv2.rectangle(frame, (int(x-w/2), int(y-h/2)), (int(x+w/2), int(y+h/2)), (255,0,0), 2) #blue thin line
                else:
                    #cone not yet detected
                    frame = cv2.rectangle(frame, (int(x-w/2), int(y-h/2)), (int(x+w/2), int(y+h/2)), (255,0,0), 20) #blue very thick line
                    
                    rectRotated = cv2.minAreaRect(contour) #bounding rectangle with minimum area around contour
                    rectRotatedBox4points = cv2.boxPoints(rectRotated) #4 points that define rectangle
                    rectRotatedBox4points = np.int0(rectRotatedBox4points)                    
                    #add the rectangle to the list
                    array_yellow_centerXYwh.append([x,y,w,h])
                    array_yellow_4points.append(rectRotatedBox4points)
                                        
                    array_detectedAll_centerXYwh.append([x,y,w,h])
                    array_detectedAll_4points.append(rectRotatedBox4points)
                    
            else:
                frame = cv2.rectangle(frame, (int(x-w/2), int(y-h/2)), (int(x+w/2), int(y+h/2)), (255,0,0), 2) #blue detected yellow area too small (cone too far away)
    return frame, array_yellow_centerXYwh, array_detectedAll_centerXYwh, array_yellow_4points, array_detectedAll_4points