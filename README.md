# Pioneer
Pioneer Robot Repository for UWA Mobile Robots.

## Pioneer Setup

## Docker

The project is encapsulated in a Docker Image. To build the image and later run a container, follow these steps:


# Image Recognition
Run the docker image as per below

Run pioneer_bringup - need the camera to be running and publishing to /oak/stereo/image_raw  
If the topic is different this needs to be changed in ./src/pioneer_image_recognition/pioneer_image_recognition/pioneer_image_recognition.py, line:
self.subscription = self.create_subscription(Image, '/oak/stereo/image_raw', self.image_cb, 10)

ros2 run pioneer_image_recognition pioneer_image_recognition

in a seperate terminal:
ros2 topic echo /number_recog

## Potential Issues:
- Add dependencies to ./src/pioneer_image_recognition/package.xml
- Dependencies for cv2

# Github
## Updating your branch to master
From a up to date branch
'''bash
git checkout master
git pull
git checkout <branch>
git rebase master
git pull
'''

## Merging your branch with master
'''bash
git add .
git commit -m '<message>'
git push
'''
[On the github](https://github.com/dumdumrobots/pioneer)
Pull requests tab
new pull request
base: master
compare: <branch>
create pull request
Deal with conflicts then merge pull request


# Docker
To run the docker image
''' bash
docker
docker compose build
docker compose up -d
docker exec -it pioneer bash
'''

### Devices major and minors listed in this file:
https://www.kernel.org/doc/Documentation/admin-guide/devices.txt

In order 
'''bash

For joy_node run ros2 run joy joy_node in one terminal and ros2 topic echo /joy in another. Pressing buttons and moving sticks can be used to determine at which location they appear in "sensor_msgs/msg/Joy".  

ros2 topic pub -r 1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"


# Pioneer Setup Summary
## 1. Github
To allow collaboration on the pioneers and for ease of transfer of work between pioneers.  
- [x]

## 2. Docker
For ease of setup of pioneers each time we work on them.  
- [x]  

## 3. Initial ROS
Pioneer is set up in the ROS environment, able to get a sim of the pioneer running.
- [x]

## 4. List out functionality and see if package exists
- [x] ARIA https://roblab.org/courses/mobrob/project/general/ariaNode.zip 
- [x] joy node: https://index.ros.org/p/joy/
- [x] teleop_twist_joy: https://index.ros.org/p/teleop_twist_joy/github-ros2-teleop_twist_joy/ 
- [] Phidgets IMU node: http://wiki.ros.org/phidgets_imu (package may not work, use serial number for code on website to program directly)
- [] Transforms broadcasters for the sensors: http://wiki.ros.org/tf
- [] Nav stack setup: http://wiki.ros.org/navigation/Tutorials/RobotSetup
- [x] Lidar - Sick Scan xd https://www.sick.com/fr/en/tim781-2174101/p/p594148 (make sure to include transform)
- [x] Camera - DepthAI API: https://docs.luxonis.com/projects/sdk/en/latest/ Code Samples: https://docs.luxonis.com/projects/api/en/latest/tutorials/code_samples/

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
=======
# Steps:  
-1: 
```sh
docker build -t pioneer .
```

-2:
```sh
docker run pioneer
```

# Checks:
Check If Container Running
```sh
docker ps
```

