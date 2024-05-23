#! /usr/bin/env python3

import rclpy
import numpy as np

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy

from rclpy.node import Node

from std_msgs.msg import Float64MultiArray, String

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener 


class LandmarkManager(Node):
    
    def __init__(self):

        super().__init__('landmark_manager')

        self.landmarks = []

        self.possible_landmarks = ["yellowCone", "redBin", "redCone", "tallCone"]

        self.landmark_subscriber = self.create_subscription(String, '/object_recog', self.landmark_callback, 10)
        self.landmark_name = 0
        self.landmark_size = 0
        self.landmark_position = [0,0]

        self.waypoint_publisher = self.create_publisher(Float64MultiArray, '/nav_waypoints', 10)
        self.landmark_publisher = self.create_publisher(Float64MultiArray, '/image_landmarks', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.landmark_publisher_timer = self.create_timer(0.1, self.landmark_publisher_timer_callback)
        self.navigator_timer = self.create_timer(0.25, self.navigator_timer_callback)

    def landmark_callback(self, msg):
        self.landmark_msg = msg
        self.landmark_name, self.landmark_size = self.landmark_msg.data.split(",")

    def landmark_publisher_timer_callback(self):

        if self.landmark_name in self.possible_landmarks and float(self.landmark_size) >= 10000:

            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform('map', 'odom', now)
            
            self.landmark_position = [transform.transform.translation.x,
                                      transform.transform.translation.y]

            self.landmarks.append(self.robot_pose)

        msg_array = np.array(self.landmark_position)

        msg = Float64MultiArray()
        msg.data = msg_array.reshape(int(msg_array.size)).tolist()

        self.landmark_publisher.publish(msg)