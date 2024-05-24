import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String
# from geometry_msgs.msg import Pose

from cv_bridge import CvBridge

import cv2
import torch
import torchvision.transforms as transforms

from datetime import datetime

import torch.nn as nn
import torch.nn.functional as F

import numpy as np


def colourRecognition(frame):
    # Define color ranges in HSV for red and yellow
    red_lower1 = np.array([0, 120, 70])
    red_upper1 = np.array([10, 255, 255])
    red_lower2 = np.array([170, 120, 70])
    red_upper2 = np.array([180, 255, 255])

    yellow_lower = np.array([20, 100, 100])
    yellow_upper = np.array([30, 255, 255])


    # Minimum contour area to be considered as a valid ROI
    min_contour_area = 15000  # Adjust this value based on your requirements
        
    # Convert the frame to HSV color space
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Create masks for red and yellow colors
    mask_red1 = cv2.inRange(hsv_frame, red_lower1, red_upper1)
    mask_red2 = cv2.inRange(hsv_frame, red_lower2, red_upper2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)
    
    mask_yellow = cv2.inRange(hsv_frame, yellow_lower, yellow_upper)
    
    # Combine the masks
    mask = cv2.bitwise_or(mask_red, mask_yellow)
    
    colours = []
    
    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for contour in contours:
        if cv2.contourArea(contour) > min_contour_area:  # Filter out small contours
            x, y, w, h = cv2.boundingRect(contour)
            
            # Determine the color of the contour
            roi_mask = mask[y:y+h, x:x+w]
            red_count = cv2.countNonZero(cv2.bitwise_and(roi_mask, mask_red[y:y+h, x:x+w]))
            yellow_count = cv2.countNonZero(cv2.bitwise_and(roi_mask, mask_yellow[y:y+h, x:x+w]))
            
            colour_label = "Red" if red_count > yellow_count else "Yellow"
            
            # Draw the bounding rectangle and the color label
            cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
            cv2.putText(frame, colour_label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)
            
            colours.append((colour_label, w*h))
            

    return colours, frame
        

"""
def computeAreaTriangle(Ax,Ay,Bx,By,Cx,Cy):
    areaOfTriangle = abs( (Bx * Ay - Ax * By) + (Cx * By - Bx * Cy) + (Ax * Cy - Cx * Ay) ) / 2
    return areaOfTriangle


def rect2cornersInRect1(rect1points,rect1area,rect2points):
    #given the four corner points of each rectangle
    #check if the corners of rectangle 2 lie in rectangle 1 
    
    A = rect1points[0]
    B = rect1points[1]
    C = rect1points[2]
    D = rect1points[3]
    Ax = A[0]
    Ay = A[1]
    Bx = B[0]
    By = B[1]
    Cx = C[0]
    Cy = C[1]
    Dx = D[0]
    Dy = D[1]
    
    intersection = False
    
    for i in range(0,4): #check for all corner points of rectangle 2 if they are inside rectangle 1#
        P = rect2points[i]
        Px = P[0]
        Py = P[1]
        areaSum = 0
        areaSum = computeAreaTriangle(Px,Py,Ax,Ay,Bx,By)+computeAreaTriangle(Px,Py,Bx,By,Cx,Cy)+computeAreaTriangle(Px,Py,Cx,Cy,Dx,Dy)+computeAreaTriangle(Px,Py,Dx,Dy,Ax,Ay)
        #areaSum = rect1area means that P is inside the rectangle or on the borderline
        if areaSum <= 1.05*rect1area: #evtl round?
            intersection = True
    
    return intersection


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
                    cv2.rectangle(frame, (int(x-w/2), int(y-h/2)), (int(x+w/2), int(y+h/2)), (255,0,0), 2) #blue thin line
                else:
                    #cone not yet detected
                    cv2.rectangle(frame, (int(x-w/2), int(y-h/2)), (int(x+w/2), int(y+h/2)), (255,0,0), 20) #blue very thick line
                    
                    rectRotated = cv2.minAreaRect(contour) #bounding rectangle with minimum area around contour
                    rectRotatedBox4points = cv2.boxPoints(rectRotated) #4 points that define rectangle
                    rectRotatedBox4points = np.int0(rectRotatedBox4points)                    
                    #add the rectangle to the list
                    array_yellow_centerXYwh.append([x,y,w,h])
                    array_yellow_4points.append(rectRotatedBox4points)
                                        
                    array_detectedAll_centerXYwh.append([x,y,w,h])
                    array_detectedAll_4points.append(rectRotatedBox4points)
                    
                    return array_yellow_centerXYwh, frame
                    
            else:
                cv2.rectangle(frame, (int(x-w/2), int(y-h/2)), (int(x+w/2), int(y+h/2)), (255,0,0), 2) #blue detected yellow area too small (cone too far away)
    #return frame, array_yellow_centerXYwh, array_detectedAll_centerXYwh, array_yellow_4points, array_detectedAll_4points


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
            cv2.drawContours(frame,[rectRotatedBox4points],0,(0,255,255),2) #cyan = could be an orange object

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
                    cv2.drawContours(frame,[rectRotatedBox4points],0,(0,255,255),2) #cyan thin line
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
                        
                        (imageh,imagew,imagec) = frame.shape
                        if x>= imagew-10 or x>= 0+10: #make sure that that not a red bin that is cut is detected as tall cone 
                            cv2.drawContours(frame,[rectRotatedBox4points],0,(0,255,255),20) #cyan very thick line
                            #add the rectangle to the list
                            array_tallCone_centerXYwh.append([x, y, w, h])
                            array_detectedAll_centerXYwh.append([x, y, w, h])
                            Box4points_tallCone.append(rectRotatedBox4points)                        
                            array_detectedAll_4points.append(rectRotatedBox4points)
                            return array_tallCone_centerXYwh, frame
                        
       
    #return frame, array_tallCone_centerXYwh, array_detectedAll_centerXYwh, Box4points_tallCone, array_detectedAll_4points


def detectRedLittleCones(frame,image_divide_redLittleCone,lower1_red,upper1_red,lower2_red,upper2_red,array_detectedAll_4points, array_detectedAll_centerXYwh):

    #initalize the arrays to store the detected objects      
    array_littleCone_centerXYwh = []
    Box4points_littleCone = [] 
  
    image_height = frame.shape[0] 
    image_width = frame.shape[1]  
    boxMinArea = image_height*image_width/image_divide_redLittleCone #adapt this: how near should the cone be s.t. we want to detect it
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
            cv2.drawContours(frame,[rectRotatedBox4points],0,(255,255,0),2) #olive green = could be an orange object

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
                    cv2.drawContours(frame,[rectRotatedBox4points],0,(255,255,0),2) #green thin line
                else:
                    #cone not yet detected
                    #check by the contour if it is really a cone (rather a triangle than a rectangle)
                    areaMinEnclosingTriangle = cv2.minEnclosingTriangle(contour)
                    # print("areaMinEnclosingTriangle")
                    # print(areaMinEnclosingTriangle[0])                   
                    areaMinEnclosingRectangle = cv2.minEnclosingTriangle(rectRotatedBox4points)
                    # print("areaMinEnclosingRectangle")
                    # print(areaMinEnclosingRectangle[0])
                    
                    #relatio w:h is max 1:4 or 4:1
                    relation_wh_Fits = True
                    if w<h:
                        if w/h < 1/4:
                            relation_wh_Fits = False
                    else:
                        if h/w < 1/4:
                            relation_wh_Fits = False
                            
                    if areaMinEnclosingTriangle[0] < 0.5*areaMinEnclosingRectangle[0] and relation_wh_Fits:
                        #rather triangle than rectangle
                        cv2.drawContours(frame,[rectRotatedBox4points],0,(255,255,0),20) #green very thick line
                        #add the rectangle to the list
                        array_littleCone_centerXYwh.append([x, y, w, h])
                        array_detectedAll_centerXYwh.append([x, y, w, h])
                        Box4points_littleCone.append(rectRotatedBox4points)                        
                        array_detectedAll_4points.append(rectRotatedBox4points)
                        return array_littleCone_centerXYwh, frame
                        
       
    #return frame, array_littleCone_centerXYwh, array_detectedAll_centerXYwh, Box4points_littleCone, array_detectedAll_4points

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
            cv2.drawContours(frame,[rectRotatedBox4points],0,(255,0,255),2) #magenta = could be an orange object

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
                    cv2.drawContours(frame,[rectRotatedBox4points],0,(255,0,255),2) #magenta thin line
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
                        cv2.drawContours(frame,[rectRotatedBox4points],0,(255,0,255),20) #magenta very thick line
                        #add the rectangle to the list
                        array_redBins_centerXYwh.append([x, y, w, h])
                        array_detectedAll_centerXYwh.append([x, y, w, h])
                        Box4points_redBins.append(rectRotatedBox4points)                        
                        array_detectedAll_4points.append(rectRotatedBox4points)
                        return array_redBins_centerXYwh, frame
                        

def detectAll(frame):
    ### eventually modify these values

    ##how near should the detected object be?
    #divide the image into areas of equal size, image_divide gives you the number of resulting areas
    #the bigger image_divide, the smaller the image can be and is still detected
    image_divide_yellow = 128
    image_divide_redLittleCone = 64
    image_divide_redBin = 16
    image_divide_TallCone = 64

    ##HSV values
    #yellow
    lower_yellow = np.array([19,190,150])
    upper_yellow = np.array([30,255,255])

    #red bins
    lower1_redBin = np.array([0, 155, 90])
    upper1_redBin = np.array([12, 255, 255])

    lower2_redBin = np.array([173,195,125])
    upper2_redBin = np.array([180,255,200])

    #tall cones
    # lower boundary red color range values;
    lower1_TallCone = np.array([0, 85, 120])
    upper1_TallCone = np.array([9, 255, 255])
    # upper boundary orange color range values
    lower2_TallCone = np.array([173,165,145])
    upper2_TallCone = np.array([180,255,255])

    #little orange cones
    # lower boundary red color range values;
    lower1_littleOrange = np.array([0, 85, 120])
    upper1_littleOrange = np.array([9, 255, 255])
    # upper boundary orange color range values
    lower2_littleOrange = np.array([173,165,145])
    upper2_littleOrange = np.array([180,255,255])


    # print(frame.shape) # heiht, width, for smartphone images 3072x4096
    
    #uncomment the next two lines if you want to detect one specific object, e.g. only tall cones:
    array_detectedAll_centerXYwh = []
    array_detectedAll_4points = []
    
    objects_detected = []
    
    #detect yellow cones
    try:
        yellowCones, frame = detectYellowCones(frame,image_divide_yellow,lower_yellow,upper_yellow)
        if yellowCones:
            for yellowCone in yellowCones:
                objects_detected.append(("yellowCone", yellowCone[2] * yellowCone[3]))
    except Exception:
        pass
    
    #detect red bins
    try:
        redBins, frame = detectRedBins(frame,image_divide_redBin,lower1_redBin,upper1_redBin,lower2_redBin,upper2_redBin,array_detectedAll_4points, array_detectedAll_centerXYwh)
        if redBins:
            for redBin in redBins:
                objects_detected.append(("redBin", redBin[2] * redBin[3]))
    except Exception:
        pass
    
    #detectTallCones
    try:
        tallCones, frame = detectTallCones(frame,image_divide_TallCone,lower1_TallCone,upper1_TallCone,lower2_TallCone,upper2_TallCone,array_detectedAll_4points, array_detectedAll_centerXYwh)
        if tallCones:
            for tallCone in tallCones:
                objects_detected.append(("tallCone", tallCone[2] * tallCone[3]))
    except Exception:
        pass
    
    # #detect little orange cones
    try:
        redCones, frame = detectRedLittleCones(frame,image_divide_redLittleCone,lower1_littleOrange,upper1_littleOrange,lower2_littleOrange,upper2_littleOrange,array_detectedAll_4points, array_detectedAll_centerXYwh)
        if redCones:
            for redCone in redCones:
                objects_detected.append(("redCone", redCone[2] * redCone[3]))
    except Exception:
        pass

    return objects_detected, frame
"""

class Pioneer(nn.Module):
    def __init__(self):
        super(Pioneer, self).__init__()
        
        # Convolutional layers
        self.conv1 = nn.Conv2d(in_channels=1, out_channels=32, kernel_size=3)
        self.conv2 = nn.Conv2d(in_channels=32, out_channels=64, kernel_size=3)
        
        # Fully connected layers
        self.fc1 = nn.Linear(in_features=64*5*5, out_features=128)
        self.fc2 = nn.Linear(in_features=128, out_features=10)

    def forward(self, x):
        # Convolutional layers with max pooling
        x = F.relu(F.max_pool2d(self.conv1(x), kernel_size=2))
        x = F.relu(F.max_pool2d(self.conv2(x), kernel_size=2))
        
        # Flatten the output for fully connected layers
        x = x.view(-1, 64*5*5)
        
        # Fully connected layers with ReLU activation
        x = F.relu(self.fc1(x))
        x = self.fc2(x)
        
        return F.log_softmax(x, dim=1)
    
class PioneerTrainer:
    def __init__(self, model, train_loader, eval_loader, criterion, optimizer, checkpoint_path):
        self.model = model
        self.train_loader = train_loader
        self.eval_loader = eval_loader
        self.criterion = criterion
        self.optimizer = optimizer
        self.checkpoint_path = checkpoint_path

    def train(self, epochs=5):
        print("Training started...")
        for epoch in range(1, epochs + 1):
            running_loss = 0.0
            for i, (inputs, labels) in enumerate(self.train_loader, 1):
                self.optimizer.zero_grad()
                outputs = self.model(inputs)
                loss = self.criterion(outputs, labels)
                loss.backward()
                self.optimizer.step()
                running_loss += loss.item()

                if i % 100 == 0:
                    print(f"Epoch [{epoch}/{epochs}], Step [{i}/{len(self.train_loader)}], Loss: {running_loss / i:.4f}")

            print(f"Epoch [{epoch}/{epochs}], Loss: {running_loss / len(self.train_loader):.4f}")

            # Save checkpoint
            self.save_checkpoint(epoch)

        print("Training finished.")

    def evaluate(self):
        print("Evaluation started...")
        correct = 0
        total = 0
        with torch.no_grad():
            for images, labels in self.eval_loader:
                outputs = self.model(images)
                _, predicted = torch.max(outputs.data, 1)
                total += labels.size(0)
                correct += (predicted == labels).sum().item()

        accuracy = 100 * correct / total
        print(f"Accuracy of the network on the {total} test images: {accuracy:.2f}%")
        print("Evaluation finished.")

    def save_checkpoint(self, epoch):
        checkpoint = {
            'epoch': epoch,
            'model_state_dict': self.model.state_dict(),
            'optimizer_state_dict': self.optimizer.state_dict(),
            'loss': self.criterion
        }
        torch.save(checkpoint, self.checkpoint_path)
        print("Checkpoint saved.")



class DigitRecogniser:
    def __init__(self, model_path='/home/ros/pioneer_ws/src/pioneer_image_recognition/pioneer_image_recognition/pioneer_checkpoint_v2.pth'):
        self.model = self.load_model(model_path)

    def load_model(self, model_path):
        # Load the checkpoint
        checkpoint = torch.load(model_path)

        # Initialize the model and load its state
        model = Pioneer()
        model.load_state_dict(checkpoint['model_state_dict'])

        # Set the model to evaluation mode
        model.eval()
        return model

    def preprocess_image(self, image):
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Resize and invert the image
        img_resized = cv2.resize(gray, (28, 28), interpolation=cv2.INTER_AREA)
        img_resized = cv2.bitwise_not(img_resized)

        # Convert to tensor and normalize
        transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize((0.5,), (0.5,))
        ])
        img_tensor = transform(img_resized)
        return img_tensor

    def predict_digit(self, img_tensor):
        with torch.no_grad():
            outputs = self.model(img_tensor.unsqueeze(0))  
            probabilities = torch.nn.functional.softmax(outputs, dim=1)
            confidence, predicted = torch.max(probabilities, 1)
        return int(predicted[0]), float(confidence * 100)

    def recognise_digit(self, image):
        digit_ROIs = []
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Apply adaptive thresholding to handle varying lighting conditions
        thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 11, 4)
        
        # Perform morphological operations to enhance the digit contours
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 5000: 
                x, y, w, h = cv2.boundingRect(cnt)
                
                # Expand the bounding box
                x -= 40  # Increase expansion in x-direction
                y -= 40  # Increase expansion in y-direction
                w += 80  # Increase expansion in width
                h += 80  # Increase expansion in height
                
                # Ensure the bounding box stays within the frame boundaries
                x = max(x, 0)
                y = max(y, 0)
                w = min(w, image.shape[1] - x)
                h = min(h, image.shape[0] - y)
                
                # Check if the contour is on a white A4 paper
                mean_color = cv2.mean(image[y:y+h, x:x+w])
                if mean_color[0] > 150 and mean_color[1] > 150 and mean_color[2] > 150:
                    digit_ROIs.append((x, y, w, h))
        return digit_ROIs

    def process_frame(self, frame):
        digits = []
        digit_ROIs = self.recognise_digit(frame)

        for x, y, w, h in digit_ROIs:
            ROI = frame[y:y+h, x:x+w]
            predicted_digit, confidence = self.predict_digit(self.preprocess_image(ROI))
            cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
            cv2.putText(frame, f"Digit: {predicted_digit}, Confidence: {confidence:.2f}%", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
            if confidence > 98:
                digits.append((predicted_digit, w * h))
                
        return digits


class Pioneer_Image_Recognition(Node):

    def __init__(self):
        super().__init__('pioneer_image_recognition')
        self.subscription = self.create_subscription(Image, '/oak/rgb/image_raw', self.image_cb, 10)
        self.number_publisher = self.create_publisher(String, '/number_recog', 10)
        self.object_publisher = self.create_publisher(String, '/object_recog', 10)
        self.image_publisher = self.create_publisher(Image, '/processed_image', 10)
        

    def image_cb(self, msg):
        self.image = msg.data
        str_msg = String()

        digit_recogniser = DigitRecogniser()
        bridge = CvBridge()

        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        try:
            digits, frame = digit_recogniser.process_frame(cv_image)
            ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            
            for digit in digits:
                now = datetime.now()
                str_msg_msg = "{},{}".format(digit[0], digit[1]) # Number, Size
                str_msg.data = str_msg_msg
                self.number_publisher.publish(str_msg)
                self.image_publisher.publish(ros_image)
                cv2.imwrite(f"/docker_shared/saved_images/{now.year}-{now.month}-{now.day}_{now.hour}.{now.minute}.{now.second}.{now.microsecond}.jpg", cv_image)
        except Exception:
            pass
        
        """
        try:
            objects, frame = detectAll(cv_image)
            ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            
            for object_ in objects:
                str_msg_msg = "{},{}".format(object_[0], object_[1]) # Number, Size
                str_msg.data = str_msg_msg
                self.object_publisher.publish(str_msg)
                self.image_publisher.publish(ros_image)
        except Exception:
            pass
        """
        
        try:
            colours, frame = colourRecognition(cv_image)
            ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            
            for colour in colours:
                now = datetime.now()
                str_msg_msg = "{},{}".format(colour[0], colour[1])
                str_msg.data = str_msg_msg
                self.object_publisher_publisher.publish(str_msg)
                self.image_publisher.publish(ros_image)
                cv2.imwrite(f"/docker_shared/saved_images/{now.year}-{now.month}-{now.day}_{now.hour}.{now.minute}.{now.second}.{now.microsecond}.jpg", cv_image)
        except Exception:
            pass


def main():
    rclpy.init()

    # Create a node that subscribes to the /joy topic and publishes to the /skidbot/cmd_vel topic.
    node = Pioneer_Image_Recognition()

    # Spin the node to receive messages and call the joy_callback function for each message.

    rclpy.spin(node)
    
    # Clean up before exiting.
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()