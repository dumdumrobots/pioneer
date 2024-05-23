import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

import numpy as np

class IntegralOdometry(Node):

    def __init__(self):
        super().__init__('int_odometry')

        self.declare_parameter('publish_tf', False)

        self.x = 0
        self.y = 0
        self.theta = 0
        
        self.delta = 0.1  # seconds
        self.timer = self.create_timer(self.delta, self.timer_callback)

        self.odom_pub = self.create_publisher(
            Odometry, 
            '/int_odom',
            10)
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel_out',
            self.cmd_vel_callback,
            10)
        
        self.cmd_vel_sub  # prevent unused variable warning
        self.cmd_vel_msg = Twist()

        self.tf_broadcaster = TransformBroadcaster(self)


    def cmd_vel_callback(self, msg):
        self.cmd_vel_msg = msg

    
    def timer_callback(self):
        
        tf_param = self.get_parameter('publish_tf').get_parameter_value().bool_value

        self.publish_odometry_msg()

        if tf_param:
            self.publish_odom_tf()


    def euler_to_quaternion(self, yaw, pitch, roll):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]
    

    def publish_odometry_msg(self):

        vx_R = self.cmd_vel_msg.linear.x
        w_R = self.cmd_vel_msg.angular.z

        self.theta += w_R * self.delta

        v_R = np.array([[vx_R],
                        [0]])

        R_iR = np.array([[np.cos(self.theta), -np.sin(self.theta)],
                         [np.sin(self.theta),  np.cos(self.theta)]])
        
        v_i = np.dot(R_iR, v_R)

        self.x += v_i[0,0] * self.delta
        self.y += v_i[1,0] * self.delta
        

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

    rough_odom_node = IntegralOdometry()

    rclpy.spin(rough_odom_node)

    rough_odom_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()