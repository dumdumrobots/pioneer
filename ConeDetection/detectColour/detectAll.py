import cv2
import matplotlib.pyplot as plt
import numpy as np
from os import listdir
from os.path import isfile, join

from fcnDetectYellowCones import detectYellowCones
from fcnDetectRedLittleCones import detectRedLittleCones
from fcnDetectRedBins import detectRedBins
from fcnDetectTallCones import detectTallCones

### eventually modify these values

##how near should the detected object be?
#divide the image into areas of equal size, image_divide gives you the number of resulting areas
#the bigger image_divide, the smaller the image can be and is still detected
image_divide_yellow = 256
image_divide_redLittleCone = 256
image_divide_redBin = 16
image_divide_TallCone = 64

##HSV values
#yellow
lower_yellow = np.array([19,160,150])
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


# input images
folder_path = "queryCones"
output_path = "output"
files = [ f for f in listdir(folder_path) if isfile(join(folder_path,f)) ]
images = np.empty(len(files), dtype=object)


for n in range(0, len(files)):
    frame = cv2.imread(join(folder_path,files[n]))
    print(files[n])
    
    #uncomment the next two lines if you want to detect one specific object, e.g. only tall cones:
    array_detectedAll_centerXYwh = []
    array_detectedAll_4points = []
    
    #detect yellow cones
    frame, array_yellow_centerXYwh, array_detectedAll_centerXYwh, array_yellow_4points, array_detectedAll_4points = detectYellowCones(frame,image_divide_yellow,lower_yellow,upper_yellow)
    print(str(len(array_yellow_centerXYwh)) + " yellow near cone(s) detected")
    
    #detect red bins
    frame, array_redBins_centerXYwh, array_detectedAll_centerXYwh, Box4points_redBins, array_detectedAll_4points = detectRedBins(frame,image_divide_redBin,lower1_redBin,upper1_redBin,lower2_redBin,upper2_redBin,array_detectedAll_4points, array_detectedAll_centerXYwh)
    print(str(len(array_redBins_centerXYwh)) + " red near bin(s) detected")
    
    #detectTallCones
    frame, array_tallCone_centerXYwh, array_detectedAll_centerXYwh, Box4points_tallCone, array_detectedAll_4points = detectTallCones(frame,image_divide_TallCone,lower1_TallCone,upper1_TallCone,lower2_TallCone,upper2_TallCone,array_detectedAll_4points, array_detectedAll_centerXYwh)
    print(str(len(array_tallCone_centerXYwh)) + " tall near cone(s) detected")
    
    # #detect little orange cones
    frame, array_littleCone_centerXYwh, array_detectedAll_centerXYwh, Box4points_littleCone, array_detectedAll_4points = detectRedLittleCones(frame,image_divide_redLittleCone,lower1_littleOrange,upper1_littleOrange,lower2_littleOrange,upper2_littleOrange,array_detectedAll_4points, array_detectedAll_centerXYwh)
    print(str(len(array_littleCone_centerXYwh)) + " red little near cone(s) detected")
                  
    cv2.imwrite(join(output_path,files[n]), frame)