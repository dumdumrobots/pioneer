#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float64MultiArray

import numpy as np

class MarkerManager(Node):
    
    def __init__(self):

        super().__init__('marker_manager')

        self.waypoints = []
        self.landmarks = []

        self.waypoint_markers = []
        self.landmark_markers = []

        self.publish_timer = self.create_timer(0.1, self.publish_timer_callback)

        self.waypoint_subscriber = self.create_subscription(Float64MultiArray, '/nav_waypoints', self.waypoint_callback, 10)
        self.landmakr_subscriber = self.create_subscription(Float64MultiArray, '/image_landmarks', self.landmark_callback, 10)

        self.markers_publisher = self.create_publisher(MarkerArray, '/markers', 10)

    def publish_timer_callback(self):

        self.waypoint_markers = self.create_marker_array(self.waypoints, type='waypoint')
        self.landmark_markers = self.create_marker_array(self.landmarks, type='landmark')

        marker_array = MarkerArray()
        marker_array.markers = self.waypoint_markers + self.landmark_markers

        #self.get_logger().info("Publishing Marker Array {0}".format(marker_array.markers))

        self.markers_publisher.publish(marker_array)
        

    def waypoint_callback(self, msg):
        msg_array = np.array(msg.data)
        self.get_logger().info("Recieved Waypoints {0}".format(msg.data))
        self.waypoints = msg_array.reshape(int(msg_array.size/2), 2).tolist()
            
    def landmark_callback(self, msg):
        msg_array = np.array(msg.data)
        self.landmarks = msg_array.reshape(int(msg_array.size/2), 2).tolist()

    def create_marker_msg(self, id, x, y, rgb=[0.0, 1.0, 0.0]):
        marker = Marker()
        marker.header.frame_id = '/map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = marker.CUBE
        marker.id = id
        marker.action = marker.ADD

        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.3

        marker.color.r = rgb[0]
        marker.color.g = rgb[1]
        marker.color.b = rgb[2]
        marker.color.a = 1.0

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.15
        
        return marker
    
    def create_marker_array(self, array, type='waypoint'):
        msg_array = []

        for index, value in enumerate(array):
            self.get_logger().info("Recieved {0}".format(value))

            if type == 'waypoint':
                rgb = [0.0, 1.0, 0.0]
                marker = self.create_marker_msg(index, value[0], value[1], rgb)

            elif type == 'landmark':
                rgb = [1.0, 0.0, 0.0]
                marker = self.create_marker_msg(index + 10, value[0], value[1], rgb)

            msg_array.append(marker)

        return msg_array
    

def main(args=None):
    rclpy.init(args=args)

    landmark = [[0.0, 0.0]]

    manager = MarkerManager()
    manager.landmarks = landmark
    
    rclpy.spin(manager)

    manager.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()