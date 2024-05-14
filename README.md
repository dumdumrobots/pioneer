# Pioneer
Pioneer Robot Repository for UWA Mobile Robots.

# Pioneer Setup Summary
## 1. Github
To allow collaboration on the pioneers and for ease of transfer of work between pioneers.  
- [x]

## 2. Docker
For ease of setup of pioneers each time we work on them.  
- []  

## 3. Initial ROS
Pioneer is set up in the ROS environment, able to get a sim of the pioneer running.
- []

## 4. List out functionality and see if package exists
- [] ARIA https://roblab.org/courses/mobrob/project/general/ariaNode.zip 
- [] joy node: https://index.ros.org/p/joy/
- [] teleop_twist_joy: https://index.ros.org/p/teleop_twist_joy/github-ros2-teleop_twist_joy/ 
- [] Phidgets IMU node: http://wiki.ros.org/phidgets_imu (package may not work, use serial number for code on website to program directly)
- [] Transforms broadcasters for the sensors: http://wiki.ros.org/tf
- [] Nav stack setup: http://wiki.ros.org/navigation/Tutorials/RobotSetup
- [] Lidar - Sick Scan xd https://www.sick.com/fr/en/tim781-2174101/p/p594148 (make sure to include transform)
- [] Camera - DepthAI API: https://docs.luxonis.com/projects/sdk/en/latest/ Code Samples: https://docs.luxonis.com/projects/api/en/latest/tutorials/code_samples/

## 5. Graph out the system with nodes, topics and message types

## 6. Test Test Test

# Task Summary

## 1. Implement Bluetooth Communication
For safety, establish Bluetooth link between robot's PC and gamepad controller for manual instruction. Utilize gamepad buttons to switch between autonomous and manual modes.

## 2. Explore Unknown Area
Initiate exploration from a set home position, mapping the unknown area as the robot moves.

## 3. Image Recognition for Hand Drawn Numbers
Detect hand-drawn numbers scattered around the environment using image recognition. Note the number and its location.

## 4. Collision Avoidance with Lidar Sensor
Utilize Lidar sensor to prevent collisions with stationary objects within the environment.

## 5. Identify and Log Yellow or Red Obstacles
Capture photos and note locations of yellow or red obstacles, as they are of special interest to the team.
As we know that all of the red and yellow obstacles are red and yellow cones, the idea is to detect the shape of cones.
If a cone is detected, determine the colour of the cone.

What we could do:
During the exploration of the unknown environment:
1. While driving around and mapping, check if there is a cone.
2. If there is a cone, drive the robot to the cone and stop in e.g. 20cm distance.
3. Determine the colour of the cone (either red or yellow).
4. Calculate the location of the cone in the real world.
5. Save the location of the cone.

### Detect the cones:
#### Var 1: SIFT/ORB:
SIFT
https://docs.opencv.org/4.x/da/df5/tutorial_py_sift_intro.html

FAST
https://docs.opencv.org/4.x/df/d0c/tutorial_py_fast.html

ORB
https://www.geeksforgeeks.org/feature-detection-and-matching-with-opencv-python/?ref=lbp

I tried a mix of the code of
https://www.geeksforgeeks.org/feature-matching-using-orb-algorithm-in-python-opencv/?ref=lbp
and
https://docs.opencv.org/4.x/dc/dc3/tutorial_py_matcher.html

It works well with the Harry Potter book (*ConeDetection/ORB/detectHarryPotter.py*).

Sadly, detecting the cones (*ConeDetection/ORB/detect_Cone.py*) did not succeed.

![Keypoints Harry Potter](ConeDetection/ORB/results/HarryPotterKeypoints.png)
![Detect Yellow Colour](ConeDetection/ORB/results/HarryPotterDetect.png)
![Cone Key Points](ConeDetection/ORB/results/ConeKeyPoints.png)
![Cone Matches Similar](ConeDetection/ORB/results/ConeMatches1.png)
![Cone Matches Many House](ConeDetection/ORB/results/ConeMatches2.png)

#### Var 2: Yolo
The code is based on https://github.com/jhan15/traffic_cones_detection/tree/master/images.

1. clone yolov5
```console
git clone https://github.com/ultralytics/yolov5  # 
cd yolov5
git reset --hard 886f1c03d839575afecb059accf74296fad395b6
```

2. install dependencies
```console
pip install -qr requirements.txt
```

3. clone https://github.com/jhan15/traffic_cones_detection/tree/master/images to another *directory2*

4. copy *model/best.pt* from *directory2* to the 1st repo in *yolov5/weights*

5. copy *utils/best.pt* from *directory2* to the 1st repo in *yolov5*

6. copy *utils/best.pt* from *directory2* to the 1st repo in *yolov5/utils*

7. Detect the cones in the pictures. Replace the last path with the path to the image.
```console
python detect.py --weights weights/best.pt --conf 0.6 --source C:\Users\49162\Documents\Uni\Master5_24_SS\MobileRobots\Project\pioneer\ConeDetection\ORB\Cones\queryCones
```
So far, *detect.py* shows the image with boxes around the cones, prints out how many cones are detected as well as the coordinates of the centres of the cones.

8. The images with boxes around the cones are saved to *runs\detect\exp*

**Results**
- Fallen cones are not detected.
- The cones that are not fallen are most of the times detected. But some of them are detected as yellow cones although they are red and some of the yellow cones are detected as green cones. --> check the colour again in a second step using the HSV value
- The tall cones are detected also as cones.
![yolov5 result1](ConeDetection/yolov5/results/IMG_20240503_154802308.jpg)
![yolov5 result 2](ConeDetection/yolov5/results/IMG_20240503_154849515.jpg)
![yolov5 result 3](ConeDetection/yolov5/results/IMG_20240503_155145312.jpg)

**To do/Questions**
- Test the code with the robot's camera.
- Ask if there are fallen cones --> yes
- Ask if there will be any other cones apart from the rather orange one and the yellow-green one --> yes: the orange tall cone and the red big bin
- Is the detection fast enough?

### Train a model with yolov5
good overview https://blog.paperspace.com/train-yolov5-custom-data/
- how to label the pictures: https://roboflow.com/formats/yolov5-pytorch-txt
Each image has one txt file with a single line for each bounding box. The format of each row is

class_id center_x center_y width height

where fields are space delimited, and the coordinates are normalized from zero to one.

Note: To convert to normalized xywh from pixel values, divide x (and width) by the image's width and divide y (and height) by the image's height.

- preprocessing of the images:
1. Auto-orient the images.

https://blog.roboflow.com/exif-auto-orientation/

How to do that? Is it necessary when using the robot's camera where we do not rotate the camera?

2. resize to 416x416

3. The following augmentation was applied to create 3 versions of each source image:
* Random rotation of between -15 and +15 degrees
* Random shear of between -10° to +10° horizontally and -10° to +10° vertically

Problem: crashes on windows, on linux not enough space.

### Detect the colour:
**Goal**: Given a the coordinates of a detected cone, decide if the cone is red or yellow.

The RGB Image is converted to an HSV Image.
Watch out, different applications use different scales for HSV. OpenCV uses H: 0-179, S: 0-255, V: 0-255.
The lower bound of red is [160,50,50] and the upper bound is [180,255,255].
The script *detectColour/detectRedColourVideo.py* detects red areas in the webcam video if they are bigger than a threshold (this threshold must be adapted for the smaller 28x28 image).

The lower bound of yellow is [20,100,100] and the upper bound is [[30,255,255].
The script *ConeDetection/detectColour/detectYellowColourVideo.py* detects yellow areas if they are bigger than a threshold.

The website https://cvexplained.wordpress.com/2020/04/28/color-detection-hsv/ also provides code for capturing an object in real time using cv2.resize.
Do we need that if we only have 28x28 pixels?

![Detect Red Colour](ConeDetection/detectColour/results/detectRedColour.png)
![Detect Yellow Colour](ConeDetection/detectColour/results/detectYellowColour.png)

The script *detectColour/detectRedColourImage.py* detects red colour of all the images located in a folder. All red cones are detected as red, no yellow cone is detect as red (successfull). But also the soil is detected as red sometimes.
![Detect Red Colour 1](ConeDetection/detectColour/results/detectRed1.jpg)
![Detect Red Colour 2](ConeDetection/detectColour/results/detectRed2.jpg)
This is why the script *detectColour/identifyHSV.py* is used to check the colour of the red cones that are rather orange than red. Run the script and double click on all the points where you want to know the HSV value. The new colour ranges are determined to [0,125,225] and [7,240,255] and show gut results. But probably they must be changed again in the future if there is different light or shadows.
![Detect Red Colour 1](ConeDetection/detectColour/results/detectRed3.jpg)
![Detect Red Colour 2](ConeDetection/detectColour/results/detectRed4.jpg)

The script *detectColour/detectYellowColourImage.py* detects yellow colour of all the images located in a folder. Some red cones are detected as yellow (not successfull). Maybe this is not too bad if we first check if the colour is red and only check for yellow colour in case no red colour was detected. Otherwise the HSV values must be adapted.
![Detect Yellow Colour 1](ConeDetection/detectColour/results/detectYellow1.jpg)
Also, the script *detectColour/detectRedColourImage.py* is used to identify the specific yellow colour ranges in the images. The new range is [20,190,210] to [30,255,255] and seems to be more reliable but it also needs to be checked with different light.
![Detect Yellow Colour 2](ConeDetection/detectColour/results/detectYellow2.jpg)


## 6. Avoid Collision with Moving Obstacles
Implement collision avoidance with moving obstacles, triggering an emergency stop if an object comes within 1m of the robot.

## 7. Save Data in Case of Emergency Stop
Upon an emergency stop, save the last 5 seconds of recorded data for review by the team.

## 8. Print Map and Marker Details
After mapping the environment, print the map along with marker photos and locations on the screen.

## 9. Implement User Interface on Robot's Display
Create a user interface displaying the robot's internal state and intended actions on the robot's display.

## 10. Plan and Navigate to Waypoints
Given three waypoints marked by hand-drawn numbers, plan the fastest path to each waypoint and back to home. Navigate to these waypoints and return home in the fastest time possible.

## 11. Display Planned Path Graphically
Show the robot's planned path graphically on a screen.

## 12. Record Drives for Offline Review
Record drives for later review offline.