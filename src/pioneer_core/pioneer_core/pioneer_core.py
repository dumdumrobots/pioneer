import rclpy
from rclpy.node import Node

import threading

from sensor_msgs.msg import Joy

BUTTON_CROSS = 0
BUTTON_CIRCLE = 1
AXIS_TRIGGER_LEFT = 4
AXIS_TRIGGER_RIGHT = 5

class Pioneer_Core(Node):

    def __init__(self):
        self.joy = Joy() # Joy msg (axes[], buttons[])
        # self.number = None # Number detection (number, location)
        # self.numbers = [] # List of numbers detected
        # self.colour = None # Colour detection (colour, location)
        # self.colours = [] # List of colours detected
        self.auto_drive = False
        self.auto_en = False
        
        super().__init__('pioneer_core')
        self.subscription = self.create_subscription(Joy, '/joy', lambda msg:joy_cb(self, msg), 10)
        # self.subscription = self.create_subscription(Number, '/number_recog', lambda number:number_cb(number), 10)
        # self.subscription = self.create_subscription(Colour, '/colour_recog', lambda colour:colour_cb(colour), 10)

    def joy_cb(self, msg):
        self.joy = msg
        if msg.buttons[BUTTON_CROSS]:
            # Enable Autonomous Mode
            self.auto_en = True
            self.get_logger().info("Enabling autonomous mode")
        
        if msg.buttons[BUTTON_CIRCLE]:
            # Enable Manual Mode
            self.auto_en = False
            self.get_logger().info("Enabling manual mode")
        
        if msg.axis[AXIS_TRIGGER_LEFT] or msg.axis[AXIS_TRIGGER_RIGHT]:
            auto_drive = True
        else:
            auto_drive = False
            
    # def num_cb(self, number):
    #     self.number = number

    # def colour_cb(self, colour):
    #     self.colour = colour
    
    def main(self):
        self.get_logger().info("Joy:")

        while self.auto_drive and self.auto_en:
            # Drive autonomously
            self.get_logger().info("Driving autonomously")

            # if len(numbers) < 9:
            #     # Explore Unknown Area

            #     if self.number:
            #         # Save number
            #         self.numbers.append(self.number)
            #         self.number = None # This may overwrite the number appended to the list
                
            #     if self.colour:
            #         # Save colour
            #         self.colours.append(self.colour)
            #         self.colour = None # This may overwrite the colour appended to the list
                
            #     # E STOP if moving object comes within 1 m

            #     # UI to show robots internal state and intended actions
            # else:
            #     # Drive to 3 waypoints
            #     pass


def main():
    rclpy.init()

    # Create a node that subscribes to the /joy topic and publishes to the /skidbot/cmd_vel topic.
    node = Pioneer_Core()

    # Spin the node to receive messages and call the joy_callback function for each message.
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    rate = node.create_rate(10)
    # If this doesn't work, remove rate and use oswait()

    try:
        while rclpy.ok():
            print("Help")
            node.main()
            rate.sleep()
    except KeyboardInterrupt:
        pass
    
    # Clean up before exiting.
    node.destroy_node()
    rclpy.shutdown()
    thread.join()


if __name__ == '__main__':
    main()