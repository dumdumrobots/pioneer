import rclpy
import numpy as np

from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy, LaserScan

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
        self.lidar_lock = True

        self.obstacle_range = 1.0472
        self.start_angle = -self.obstacle_range/2
        self.end_angle = self.obstacle_range/2

        self.lidar_msg = LaserScan()
        self.lidar_min_value = 0
        
        self.joy_subscriber = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.lidar_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.man_lock_publisher = self.create_publisher(Bool, '/pause_man', 10)
        self.nav_lock_publisher = self.create_publisher(Bool, '/pause_nav', 10)
        self.lidar_lock_publisher = self.create_publisher(Bool, '/stop_all', 10)

        self.interlocking_timer = self.create_timer(0.1, self.timer_callback)


    def timer_callback(self):

         # --- Lidar Switch

        self.data = np.array(self.lidar_msg.ranges)
        self.angle_min = self.lidar_msg.angle_min
        self.increment = self.lidar_msg.angle_increment
        

        if self.increment != 0.0:

            self.start_index = int((self.start_angle - self.angle_min)/ self.increment)
            self.end_index = int((self.end_angle - self.angle_min)/ self.increment)

            valid_array = []

            for value in self.data[self.start_index:self.end_index]:
                if value >= 0.05:
                    valid_array.append(value)

            self.lidar_min_value = np.min(np.array(valid_array))

        if self.lidar_min_value <= 0.5:
            self.lidar_lock  = True
        else:
            self.lidar_lock  = False

        self.get_logger().info("Minimal LiDAR Value: {0}\n".format(self.lidar_min_value))

        # --- Button Switches

        bool_man = Bool()
        bool_nav = Bool()
        bool_lidar = Bool()

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
        bool_man.data = self.manual_lock or self.lidar_lock
        bool_lidar.data = self.lidar_lock
            
        self.nav_lock_publisher.publish(bool_nav)
        self.man_lock_publisher.publish(bool_man)
        self.lidar_lock_publisher.publish(bool_lidar)

        self.joy_buttons_last = self.joy_buttons

        self.get_logger().info("Interlocking Status:\nAutonomous: {0} Manual: {1} Deadman Trigger: {2} LiDAR Trigger: {3}\n".format(
            self.autonomous_lock, self.manual_lock, self.dead_trigger, self.lidar_lock))


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