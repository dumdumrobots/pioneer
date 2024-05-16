import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
# from geometry_msgs.msg import Pose

from ROI import DigitRecogniser

from ImageRecognition import *

from cv_bridge import CvBridge

class Pioneer_Image_Recognition(Node):

    def __init__(self):
        # self.pose = Pose()
        super().__init__('pioneer_image_recognition')
        self.subscription = self.create_subscription(Image, '/oak/stereo/image_raw', self.image_cb, 10)
        # self.subscription = self.create_subscription(Pose, '/odom', self.pose_cb, 10)
        self.publisher_ = self.create_publisher(Number, '/number_recog', 10)
        # self.publisher_ = self.create_publisher(Colour, '/colour_recog', 10)


    def image_cb(self, msg):
        self.image = msg.data

        digit_recogniser = DigitRecogniser()
        bridge = CvBridge()

        cv_image = bridge.imgmsg_to_cv2(self.image, desired_encoding='passthrough')

        digits = digit_recogniser.process_frame(cv_image)

        for digit in digits:
            number = Number(digit[0], digit[1]) # Number, Size, Pose
            self.publisher_.publish(number)


    # def pose_cb(self, msg):ssssssssssssssssssssssssssssssssssssss
    #     self.pose = msg.position


def main():
    rclpy.init()

    # Create a node that subscribes to the /joy topic and publishes to the /skidbot/cmd_vel topic.
    node = Pioneer_Image_Recognition()

    # Spin the node to receive messages and call the joy_callback function for each message.

    rclpy.spin(node)
    
    # Clean up before exiting.
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()