import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import numpy as np

class RoughOdometry(Node):

    def __init__(self):
        super().__init__('rough_odometry')

        self.curret_time = self.get_clock().now().nanoseconds
        self.last_time = self.get_clock().now().nanoseconds

        self.x = 0
        self.y = 0
        self.theta = 0


        self.publisher_ = self.create_publisher(
            Odometry, 
            '/command_odom',
            10)
        
        self.subscriber_ = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_sub_callback,
            10)
        
        self.subscriber_  # prevent unused variable warning


    def cmd_sub_callback(self,msg):

        cmd_vel = msg

        self.curret_time = self.get_clock().now().nanoseconds

        delta = (self.curret_time - self.last_time) * 1e-9

        vx = cmd_vel.linear.x
        vy = cmd_vel.linear.y
        w = cmd_vel.angular.z

        self.x += vx * delta
        self.y += vy * delta
        self.theta += w * delta

        pub_msg = Odometry()

        pub_msg.pose.pose.position.x = self.x
        pub_msg.pose.pose.position.y = self.y

        result_quaternion = self.euler_to_quaternion(self.theta, 0, 0)
        pub_msg.pose.pose.orientation.x = result_quaternion[0]
        pub_msg.pose.pose.orientation.y = result_quaternion[1]
        pub_msg.pose.pose.orientation.z = result_quaternion[2]
        pub_msg.pose.pose.orientation.w = result_quaternion[3]

        pub_msg.twist.twist = msg
        
        self.publisher_.publish(pub_msg)

        self.last_time = self.curret_time


    def euler_to_quaternion(self, yaw, pitch, roll):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]


def main(args=None):
    rclpy.init(args=args)

    rough_odom_node = RoughOdometry()

    rclpy.spin(rough_odom_node)

    rough_odom_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()