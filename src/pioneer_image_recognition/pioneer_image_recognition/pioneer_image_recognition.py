import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose

from ROI import DigitRecogniser

class Pioneer_Core(Node):

    def __init__(self):
        self.joy = None # Joy msg (axes[], buttons[])
        self.number = None # Number detection (number, location)
        self.numbers = [] # List of numbers detected
        self.colour = None # Colour detection (colour, location)
        self.colours = [] # List of colours detected
        self.auto_drive = False
        super().__init__('pioneer_core')
        self.subscription = self.create_subscription(Joy, '/joy', lambda msg:joy_cb(self, msg), 10)
        self.subscription = self.create_subscription(Number, '/number_recog', lambda number:number_cb(number), 10)
        self.subscription = self.create_subscription(Colour, '/colour_recog', lambda colour:colour_cb(colour), 10)

    def joy_cb(self, msg):
        self.joy_data = msg

    def num_cb(self, number):
        self.number = number

    def colour_cb(self, colour):
        self.colour = colour
    
    def main(self):
        if self.joy.buttons[BUTTON_CROSS]:
            # Enable autonomous mode
            self.auto_drive = True
            self.get_logger().info("Enabling autonomous mode")
        
        if self.joy.buttons[BUTTON_CIRCLE]:
            # Enable manual mode
            self.auto_drive = False
            self.get_logger().info("Enabling manual mode")

        while self.auto_drive and (self.joy.axis[AXIS_TRIGGER_LEFT] or self.joy.axis[AXIS_TRIGGER_RIGHT]):
            # Drive autonomously
            self.get_logger().info("Driving autonomously")

            if len(numbers) < 9:
                # Explore Unknown Area

                if self.number:
                    # Save number
                    self.numbers.append(self.number)
                    self.number = None # This may overwrite the number appended to the list
                
                if self.colour:
                    # Save colour
                    self.colours.append(self.colour)
                    self.colour = None # This may overwrite the colour appended to the list
                
                # E STOP if moving object comes within 1 m

                # UI to show robots internal state and intended actions
            else:
                # Drive to 3 waypoints


def main():
    rclpy.init()

    # Create a node that subscribes to the /joy topic and publishes to the /skidbot/cmd_vel topic.
    node = Pioneer_Core()
    # subscription = node.create_subscription(Joy, '/joy', lambda msg: joy_callback(msg), 10)
    # subscription2 = node.create_subscription()
    # publisher = node.create_publisher(Twist, '/skidbot/cmd_vel', 10)

    # Spin the node to receive messages and call the joy_callback function for each message.

    rate = node.create_rate(10)

    while rclpy.ok()
        node.main()
        rate.sleep()
    
    # Clean up before exiting.
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()