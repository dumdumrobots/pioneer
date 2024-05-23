import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from cv_bridge import CvBridge
import os

import cv2
import numpy as np

from datetime import datetime
import subprocess


class Pioneer_Rosbags(Node):

    def __init__(self):
        super().__init__('pioneer_rosbags')

        self.image_subscription = self.create_subscription(Image, '/oak/rgb/image_raw', self.image_cb, 10)
        self.stopall_subscription = self.create_subscription(Bool, '/stop_all', self.stopall_cb, 10)
        self.image_publisher = self.create_publisher(Image, '/stopall_images', 10)

        self.bridge = CvBridge()
        self.stopped = False

        self.images = []
        os.mkdir("/home/ros/pioneer_ws/rosbags/", 0o777)
        self.bag_process = None
        self.start_rosbags()

    def image_cb(self, msg):
        if not self.stopped:
            now = datetime.now()
            if len(images) > 50: # 10 images / second therefore 50 images in 50 seconds
                self.images.pop(1) # Remove earliest image
                self.images.append(msg, f"{now.year}-{now.month}-{now.day}_{now.hour}.{now.minute}.{now.second}.{now.microsecond}") # Append new images
            else:
                self.images.append(msg, f"{now.year}-{now.month}-{now.day}_{now.hour}.{now.minute}.{now.second}.{now.microsecond}")

    def stopall_cb(self, msg):
        if msg.data: # Stop All True
            self.stopped = True
            # Save last 5 seconds of images
            os.mkdir("/home/ros/pioneer_ws/stopall_images", 0o777)
            for image in self.images:
                self.image_publisher.publish(image[0])
                cv2_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                cv2.imwrite(f"/home/ros/pioneer_ws/stopall_images/{image[1]}.jpg", cv2_image)
            self.stop_rosbags()

    def start_rosbags(self):
        self.bag_process = subprocess.Popen(['ros2', 'bag', 'record', '/home/ros/pioneer_ws/rosbags/', '-a'])

    def stop_rosbags(self):
        self.bag_process.terminate()
        self.bag_process.wait()


def main():
    rclpy.init()

    # Create a node that subscribes to the /joy topic and publishes to the /skidbot/cmd_vel topic.
    node = Pioneer_Image_Recognition()

    # Spin the node to receive messages and call the joy_callback function for each message.

    rclpy.spin(node)
    
    # Clean up before exiting.
    node.stop_rosbags()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()