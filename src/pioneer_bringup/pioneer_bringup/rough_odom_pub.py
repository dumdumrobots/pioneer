import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class RoughOdometry(Node):

    def __init__(self):
        super().__init__('rough_odometry')

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
        print(msg)

        pub_msg = Odometry()
        pub_msg.twist.twist = msg

        
        self.publisher_.publish(pub_msg)


def main(args=None):
    rclpy.init(args=args)

    rough_odom_node = RoughOdometry()

    rclpy.spin(rough_odom_node)

    rough_odom_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()