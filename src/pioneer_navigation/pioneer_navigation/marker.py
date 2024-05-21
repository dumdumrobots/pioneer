#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

class Marker(Node):
    def __init__(self,  id, name, x, y, z):

        super().__init__(name)
        self.publisher_ = self.create_publisher(Marker, name, 10)  

        self.marker = Marker()
        self.marker.header.frame_id = '/map'
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker.type = self.marker.CUBE
        self.marker.id = id
        self.marker.action = self.marker.ADD

        self.marker.scale.x = 0.5
        self.marker.scale.y = 0.5
        self.marker.scale.z = 0.5
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0

        self.marker.pose.position.x = x
        self.marker.pose.position.y = y
        self.marker.pose.position.z = z
        self.get_logger().info("Publishing marker topic.")

    def publish_marker(self):
        self.publisher_.publish(self.marker)

def main(args=None):
    rclpy.init(args=args)
    
    balloon = Marker()

    while rclpy.ok():
        balloon.publish_marker()

    balloon.destroy_node()  
    rclpy.shutdown()

if __name__ == '__main__':
    main()