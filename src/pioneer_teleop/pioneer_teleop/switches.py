import rclpy
import threading

from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy

BUTTON_CROSS = 0
BUTTON_CIRCLE = 1

AXIS_TRIGGER_LEFT = 6
AXIS_TRIGGER_RIGHT = 7

class Switches(Node):

    def __init__(self):
        super().__init__('switches')

        self.joy_buttons = [0,0,0,0,0,0,0,0,0]
        self.joy_buttons_last = [0,0,0,0,0,0,0,0,0]

        self.autonomous_lock = True
        self.manual_lock = True

        self.dead_trigger = True

        self.get_logger().info("Current lock status \n Autonomous: {0} \n Manual: {1}".format(self.autonomous_lock, self.manual_lock))
        
        self.joy_subscriber = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.man_lock_publisher = self.create_publisher(Bool, '/pause_man', 10)
        self.nav_lock_publisher = self.create_publisher(Bool, '/pause_nav', 10)

        self.interlocking_timer = self.create_timer(0.1, self.timer_callback)


    def timer_callback(self):

        bool_man = Bool()
        bool_nav = Bool()

        if self.joy_buttons[AXIS_TRIGGER_LEFT] == 1:
            self.dead_trigger = False
        else:
            self.dead_trigger = True

        if ((self.joy_buttons[BUTTON_CIRCLE] != self.joy_buttons_last[BUTTON_CIRCLE]) 
            and self.joy_buttons[BUTTON_CIRCLE] == 1):
            self.autonomous_lock = not self.autonomous_lock
            
            
        if ((self.joy_buttons[BUTTON_CROSS] != self.joy_buttons_last[BUTTON_CROSS]) 
            and self.joy_buttons[BUTTON_CROSS] == 1):
            self.manual_lock = not self.manual_lock

        bool_nav.data = self.autonomous_lock or self.dead_trigger
        bool_man.data = self.manual_lock
            
        self.nav_lock_publisher.publish(bool_nav)
        self.man_lock_publisher.publish(bool_man)

        self.joy_buttons_last = self.joy_buttons

        self.get_logger().info("Interlocking Status: \nAutonomous: {0} \nDeadman Trigger: {1} \nManual: {2}".format(self.autonomous_lock, self.dead_trigger, self.manual_lock))


    def joy_callback(self, msg):
        self.joy_buttons = msg.buttons


def main():
    rclpy.init()

    switches = Switches()

    rclpy.spin(switches)
    
    # Clean up before exiting.
    switches.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()