import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

import numpy as np

class RoughOdometry(Node):

    def __init__(self):
        super().__init__('rough_odometry')

        self.curret_time = self.get_clock().now().nanoseconds
        self.last_time = self.get_clock().now().nanoseconds

        self.x = 0
        self.y = 0
        self.theta = 0


        self.odom_pub = self.create_publisher(
            Odometry, 
            '/i_odom',
            10)
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_sub_callback,
            10)
        
        self.cmd_vel_sub  # prevent unused variable warning

        self.tf_broadcaster = TransformBroadcaster(self)


    def cmd_vel_sub_callback(self,msg):
        self.publish_odometry_msg(msg)
        self.publish_odom_tf()
        


    def euler_to_quaternion(self, yaw, pitch, roll):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]
    

    def publish_odometry_msg(self, twist_msg):

        self.curret_time = self.get_clock().now().nanoseconds

        delta = (self.curret_time - self.last_time) * 1e-9

        vx_R = twist_msg.linear.x
        w_R = twist_msg.angular.z

        self.theta += w_R * delta

        v_R = np.array([[vx_R],
                        [0]])

        R_iR = np.array([[np.cos(self.theta), -np.sin(self.theta)],
                         [np.sin(self.theta),  np.cos(self.theta)]])
        
        v_i = np.dot(R_iR, v_R)

        self.x += v_i[0,0] * delta
        self.y += v_i[1,0] * delta
        

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        self.q = self.euler_to_quaternion(self.theta, 0, 0)
        odom.pose.pose.orientation.x = self.q[0]
        odom.pose.pose.orientation.y = self.q[1]
        odom.pose.pose.orientation.z = self.q[2]
        odom.pose.pose.orientation.w = self.q[3]

        odom.twist.twist.linear.x = v_i[0,0]
        odom.twist.twist.linear.x = v_i[1,0]

        odom.twist.twist.angular.z = w_R

        self.odom_pub.publish(odom)
        self.last_time = self.curret_time
    
    
    def publish_odom_tf(self):

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = self.q[0]
        t.transform.rotation.y = self.q[1]
        t.transform.rotation.z = self.q[2]
        t.transform.rotation.w = self.q[3]

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)

    rough_odom_node = RoughOdometry()

    rclpy.spin(rough_odom_node)

    rough_odom_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()