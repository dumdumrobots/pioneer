import cv2
import matplotlib.pyplot as plt
import numpy as np
from os import listdir
from os.path import isfile, join

from fcn_rect2cornersInRect1 import rect2cornersInRect1

def detectRedBins(frame,image_divide_redBin,lower1_red,upper1_red,lower2_red,upper2_red,array_detectedAll_4points, array_detectedAll_centerXYwh):

    #initalize the arrays to store the detected objects      
    array_redBins_centerXYwh = []
    Box4points_redBins = [] 
  
    image_height = frame.shape[0] 
    image_width = frame.shape[1]  
    boxMinArea = image_height*image_width/image_divide_redBin #adapt this: how near should the bin be s.t. we want to detect it
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
            frame = cv2.drawContours(frame,[rectRotatedBox4points],0,(255,0,255),2) #magenta = could be an orange object

            if(w*h>boxMinArea):  #bin must be near                  
                binAlreadyDetected = False
                for kk in range(0,len(array_detectedAll_centerXYwh)):
                    XYwh_existing = array_detectedAll_centerXYwh[kk]
                    w_existing = XYwh_existing[2]
                    h_existing = XYwh_existing[3]
                    rectRotatedBox4points_existing = array_detectedAll_4points[kk]

                    if rect2cornersInRect1(rectRotatedBox4points,w*h,rectRotatedBox4points_existing) or rect2cornersInRect1(rectRotatedBox4points_existing,w_existing*h_existing,rectRotatedBox4points):
                        #bin already detected
                        binAlreadyDetected = True
                                   
                if binAlreadyDetected == True:
                    #bin already detected
                    frame = cv2.drawContours(frame,[rectRotatedBox4points],0,(255,0,255),2) #magenta thin line
                else:
                    #bin not yet detected
                    #check by the contour if it is really a bin (rather a rectangle than a triangle)
                    areaMinEnclosingTriangle = cv2.minEnclosingTriangle(contour)
                    # print("areaMinEnclosingTriangle")
                    # print(areaMinEnclosingTriangle[0])                   
                    areaMinEnclosingRectangle = cv2.minEnclosingTriangle(rectRotatedBox4points)
                    # print("areaMinEnclosingRectangle")
                    # print(areaMinEnclosingRectangle[0])
                    
                    if areaMinEnclosingTriangle[0] > 0.5*areaMinEnclosingRectangle[0]:
                        #rather rectangle than triangle
                        frame = cv2.drawContours(frame,[rectRotatedBox4points],0,(255,0,255),20) #magenta very thick line
                        #add the rectangle to the list
                        array_redBins_centerXYwh.append([x, y, w, h])
                        array_detectedAll_centerXYwh.append([x, y, w, h])
                        Box4points_redBins.append(rectRotatedBox4points)                        
                        array_detectedAll_4points.append(rectRotatedBox4points)
                        
       
    return frame, array_redBins_centerXYwh, array_detectedAll_centerXYwh, Box4points_redBins, array_detectedAll_4points