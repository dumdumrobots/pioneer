#! /usr/bin/env python3

import rclpy
import numpy as np
import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy

from rclpy.node import Node

from std_msgs.msg import Float64MultiArray, String

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener 


class ObjectManager(Node):
    
    def __init__(self):

        super().__init__('object_manager')

        self.landmarks = []
        self.numbers = []
        self.registered_numbers = []

        self.possible_landmarks = ["Yellow", "Red"]
        self.possible_numbers = ["0","1","2","3","4","5","6","7","8","9"]

        self.landmark_subscriber = self.create_subscription(String, '/object_recog', self.landmark_callback, 10)
        self.number_subscriber = self.create_subscription(String, '/number_recog', self.number_callback, 10)

        self.landmark_name = 0
        self.landmark_size = 0
        self.landmark_position = [0,0]

        self.number_name = 0
        self.number_size = 0
        self.number_position = [0,0]

        self.landmark_publisher = self.create_publisher(Float64MultiArray, '/image_landmarks', 10)
        self.number_publisher = self.create_publisher(Float64MultiArray, '/image_numbers', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publisher_timer = self.create_timer(0.1, self.publisher_timer_callback)


    def get_object_position(self):
        now = rclpy.time.Time()
        transform = self.tf_buffer.lookup_transform('map', 'odom', now)

        x = transform.transform.rotation.x
        y = transform.transform.rotation.y
        z = transform.transform.rotation.z
        w = transform.transform.rotation.w

        roll_x, pitch_y, yaw_z = self.euler_from_quaternion(x,y,z,w)

        R_iR = np.array([[np.cos(yaw_z), -np.sin(yaw_z)],
                        [np.sin(yaw_z),  np.cos(yaw_z)]])
        
        p_R = np.array([[1],
                        [0]])

        p_i = np.dot(R_iR, p_R)

        object_position = [transform.transform.translation.x + p_i[0,0],
                            transform.transform.translation.y + p_i[1,0]]
        
        return object_position


    def landmark_callback(self, msg):
        landmark_msg = msg
        self.landmark_name, self.landmark_size = landmark_msg.data.split(",")


    def number_callback(self, msg):
        number_msg = msg
        self.number_name, self.number_size = number_msg.data.split(",")

    def calculate_distance(self, position_1, position_2):

        tGC = position_1 - position_2
        distance = np.sqrt(np.power(tGC[0],2) + np.power(tGC[1],2))
        
        return distance 



    def publisher_timer_callback(self):

        # --- Append landmarks

        if self.landmark_name in self.possible_landmarks and float(self.landmark_size) >= 90000:

            self.landmark_position = self.get_object_position()
            close_condition = False

            for existing_landmark in self.landmarks:

                distance = self.calculate_distance(self.landmark_position,
                                                  existing_landmark)
                
                if distance <= 1.5:
                    close_condition = True
                    break

            if close_condition == False:
                self.landmarks.append(self.landmark_position)

        # --- Append numbers

        if ((self.number_name in self.possible_numbers) and
            not (self.number_name in self.registered_numbers) and
            (float(self.number_size) >= 50000)):

            self.number_position = self.get_object_position()
            self.numbers.append(self.number_position)
            self.registered_numbers.append(self.number_name)

        # --- Publish landmarks

        msg_array_landmarks = np.array(self.landmarks)

        msg_landmarks = Float64MultiArray()
        msg_landmarks.data = msg_array_landmarks.reshape(int(msg_array_landmarks.size)).tolist()

        self.landmark_publisher.publish(msg_landmarks)

        # --- Publish numbers

        msg_array_numbers = np.array(self.numbers)

        msg_numbers = Float64MultiArray()
        msg_numbers.data = msg_array_numbers.reshape(int(msg_array_numbers.size)).tolist()

        self.number_publisher.publish(msg_numbers)


    def euler_from_quaternion(self, x, y, z, w):

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians
    

def main():
    rclpy.init()
    
    landmark = ObjectManager()

    rclpy.spin(landmark)

    landmark.navigator.lifecycleShutdown()
    landmark.destroy_node()

    exit(0)

if __name__ == '__main__':
    main()