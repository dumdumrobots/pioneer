import cv2
import matplotlib.pyplot as plt
import numpy as np
from os import listdir
from os.path import isfile, join

from fcn_rect2cornersInRect1 import rect2cornersInRect1

def detectTallCones(frame,image_divide_TallCone,lower1_red,upper1_red,lower2_red,upper2_red,array_detectedAll_4points, array_detectedAll_centerXYwh):

    #initalize the arrays to store the detected objects      
    array_tallCone_centerXYwh = []
    Box4points_tallCone = [] 
  
    image_height = frame.shape[0] 
    image_width = frame.shape[1]  
    boxMinArea = image_height*image_width/image_divide_TallCone #adapt this: how near should the cone be s.t. we want to detect it
    MinArea = boxMinArea/100

    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    #check if the HSV of the frame is lower or upper red
    lower_mask = cv2.inRange(hsv, lower1_red, upper1_red)
    upper_mask = cv2.inRange(hsv, lower2_red, upper2_red)
    mask = cv2.bitwise_xor(lower_mask, upper_mask) 
        
    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame,frame, mask= mask)

    #check how big the area is
    (contours, hierarchy) = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for pic,contour in enumerate(contours):
        area = cv2.contourArea(contour) #how big is the area of the contour
        
        if(area > MinArea): #to remove the noise            
            rectRotated = cv2.minAreaRect(contour) #bounding rectangle with minimum area around contour
            #( center (x,y), (width, height), angle of rotation )
            centerRectRotated = rectRotated[0]
            x = int(centerRectRotated[0])
            y = int(centerRectRotated[1])
            widthHeightRectRotated = rectRotated[1]
            w = widthHeightRectRotated[0]
            h = widthHeightRectRotated[1]
            rectRotatedBox4points = cv2.boxPoints(rectRotated) #4 points that define rectangle
            rectRotatedBox4points = np.int0(rectRotatedBox4points)
            frame = cv2.drawContours(frame,[rectRotatedBox4points],0,(0,255,255),2) #cyan = could be an orange object

            if(w*h>boxMinArea):  #cone must be near                  
                coneAlreadyDetected = False
                for kk in range(0,len(array_detectedAll_centerXYwh)):
                    XYwh_existing = array_detectedAll_centerXYwh[kk]
                    w_existing = XYwh_existing[2]
                    h_existing = XYwh_existing[3]
                    rectRotatedBox4points_existing = array_detectedAll_4points[kk]

                    if rect2cornersInRect1(rectRotatedBox4points,w*h,rectRotatedBox4points_existing) or rect2cornersInRect1(rectRotatedBox4points_existing,w_existing*h_existing,rectRotatedBox4points):
                        #cone already detected
                        coneAlreadyDetected = True
                                   
                if coneAlreadyDetected == True:
                    #cone already detected
                    frame = cv2.drawContours(frame,[rectRotatedBox4points],0,(0,255,255),2) #cyan thin line
                else:
                    #cone not yet detected
                    #check by the contour if it is really a tall cone (rather a rectangle than a triangle)
                    areaMinEnclosingTriangle = cv2.minEnclosingTriangle(contour)
                    # print("areaMinEnclosingTriangle")
                    # print(areaMinEnclosingTriangle[0])                   
                    areaMinEnclosingRectangle = cv2.minEnclosingTriangle(rectRotatedBox4points)
                    # print("areaMinEnclosingRectangle")
                    # print(areaMinEnclosingRectangle[0])
                    
                    #relatio w:h is bigger than 1:4 or 4:1
                    relation_wh_Fits = True
                    if w<h:
                        if w/h > 1/4:
                            relation_wh_Fits = False
                    else:
                        if h/w > 1/4:
                            relation_wh_Fits = False
                            
                    if areaMinEnclosingTriangle[0] > 0.5*areaMinEnclosingRectangle[0] and relation_wh_Fits:
                        #rather triangle than rectangle
                        frame = cv2.drawContours(frame,[rectRotatedBox4points],0,(0,255,255),20) #cyan very thick line
                        #add the rectangle to the list
                        array_tallCone_centerXYwh.append([x, y, w, h])
                        array_detectedAll_centerXYwh.append([x, y, w, h])
                        Box4points_tallCone.append(rectRotatedBox4points)                        
                        array_detectedAll_4points.append(rectRotatedBox4points)
                        
       
    return frame, array_tallCone_centerXYwh, array_detectedAll_centerXYwh, Box4points_tallCone, array_detectedAll_4points