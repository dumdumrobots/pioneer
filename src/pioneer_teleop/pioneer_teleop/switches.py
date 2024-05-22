import rclpy
import numpy as np

from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy, LaserScan

BUTTON_CROSS = 0
BUTTON_CIRCLE = 1

AXIS_TRIGGER_LEFT = 6
AXIS_TRIGGER_RIGHT = 7

class LiDAR(Node):
    def __init__(self, laser_msg):
        self.obstacle_range = 1.0472
        self.data = np.array(laser_msg.ranges)

        self.start_angle = -self.obstacle_range/2
        self.end_angle = self.obstacle_range/2

        self.increment = laser_msg.angle_increment

        self.start_index = int((self.start_angle - -self.obstacle_range/2)/ self.increment)
        self.end_index = int((self.start_angle - -self.obstacle_range/2)/ self.increment)

    def check 






class Switches(Node):

    def __init__(self):
        super().__init__('switches')

        self.joy_buttons = [0,0,0,0,0,0,0,0,0]
        self.joy_buttons_last = [0,0,0,0,0,0,0,0,0]

        self.autonomous_lock = True
        self.manual_lock = True

        self.dead_trigger = True
        self.lidar_trigger = True

        self.obstacle_range = 1.0472
        self.start_angle = -self.obstacle_range/2
        self.end_angle = self.obstacle_range/2

        self.lidar_msg = LaserScan()


        self.get_logger().info("Current lock status \n Autonomous: {0} \n Manual: {1}".format(self.autonomous_lock, self.manual_lock))
        
        self.joy_subscriber = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.lidar_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.man_lock_publisher = self.create_publisher(Bool, '/pause_man', 10)
        self.nav_lock_publisher = self.create_publisher(Bool, '/pause_nav', 10)

        self.interlocking_timer = self.create_timer(0.1, self.timer_callback)


    def timer_callback(self):

         # --- Lidar Switch

        self.data = np.array(self.lidar_msg.ranges)
        self.increment = self.lidar_msg.angle_increment

        self.start_index = int((self.start_angle - -self.obstacle_range/2)/ self.increment)
        self.end_index = int((self.start_angle - -self.obstacle_range/2)/ self.increment)

        min_value = np.min(self.data[self.start_index, self.end_index])

        if min_value > 0 and  min_value <= 5:
            self.lidar_trigger  = True
        else:
            self.lidar_trigger  = False

        self.get_logger().info("Minimal LiDAR Value: {0} ".format(min_value))

        # --- Button Switches

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

        self.get_logger().info("Interlocking Status: \nAutonomous: {0} \nManual: {1} \nDeadman Trigger: {2} \nLiDAR Trigger: {3} ".format(
            self.autonomous_lock, self.manual_lock, self.dead_trigger, self.lidar_trigger))


    def joy_callback(self, msg):
        self.joy_buttons = msg.buttons

    def scan_callback(self, msg):
        self.lidar_msg = msg


def main():
    rclpy.init()

    switches = Switches()

    rclpy.spin(switches)
    
    # Clean up before exiting.
    switches.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()