import rclpy
import threading

from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy

BUTTON_CROSS = 0
BUTTON_CIRCLE = 1
AXIS_TRIGGER_LEFT = 4
AXIS_TRIGGER_RIGHT = 5

class Switches(Node):

    def __init__(self):
        super().__init__('switches')

        self.joy_buttons = []
        self.joy_axes = []

        self.autonomous_lock = True
        self.manual_lock = True

        self.get_logger().info("Current lock status \n Autonomous: {0} \n Manual: {1}".format(self.autonomous_lock, self.manual_lock))
        
        self.joy_subscriber = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.man_lock_publisher = self.create_publisher(Bool, '/pause_man', 10)
        self.nav_lock_publisher = self.create_publisher(Bool, '/pause_nav', 10)

        self.interlocking_timer = self.create_timer(1, self.timer_callback)


    def timer_callback(self):

        if self.joy_buttons[BUTTON_CROSS]:
            self.autonomous_lock = not self.autonomous_lock
            self.autonomous_lock_publisher.publish(self.autonomous_lock)
        
        if self.joy_buttons[BUTTON_CIRCLE]:
            self.manual_lock = not self.autonomous_lock
            self.man_lock_publisher.publish(self.manual_lock)

        self.get_logger().info("Current LOCK status \n Autonomous: {0} \n Manual: {1}".format(self.autonomous_lock, self.manual_lock))


    def joy_callback(self, msg):
        self.joy_buttons = msg.buttons
        self.joy_axes = msg.axes


def main():
    rclpy.init()

    switches = Switches()

    rclpy.spin(switches)
    
    # Clean up before exiting.
    switches.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()