#! /usr/bin/env python3

import rclpy
import numpy as np

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from rclpy.duration import Duration
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray, String

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener 
from tf2_ros import TransformException

BUTTON_TRIANGLE = 2
BUTTON_SQUARE = 3

class WaypointManager(Node):
    
    def __init__(self):

        super().__init__('waypoint_manager')

        self.navigator = BasicNavigator()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.joy_buttons = [0,0,0,0,0,0,0,0,0]
        self.joy_buttons_last = [0,0,0,0,0,0,0,0,0]

        self.goal_waypoints = []
        self.goal_poses = []

        self.current_goal_pose = [0.0,0.0]

        self.robot_pose, self.robot_position = self.create_pose(x= 0.0,y= 0.0,w= 1.0,z= 0.0)
        self.current_goal_pose = self.robot_pose

        self.navigator.setInitialPose(self.robot_pose)
        self.nav_start = self.navigator.get_clock().now()
        self.now = self.navigator.get_clock().now()

        self.joy_subscriber = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.waypoint_publisher = self.create_publisher(Float64MultiArray, '/nav_waypoints', 10)
        self.overlay_publisher = self.create_publisher(String, '/input_text', 10)

        self.waypoint_publisher_timer = self.create_timer(0.1, self.waypoint_publisher_timer_callback)
        self.navigator_timer = self.create_timer(0.25, self.navigator_timer_callback)

        self.waypoint_manager_states = {
            0 : "Stand-by",
            1 : "Recording",
            2 : "Setup",
            3 : "Navigating",
            4 : "End",
        }

        self.current_state = self.waypoint_manager_states[0]

        self.get_logger().info("Entering Navigator initial {0} state.\n".format(self.current_state))

        

    def reset_goal_arrays(self):
        self.goal_waypoints = []
        self.goal_poses = []
        self.goal_index = 0

    def get_robot_pose(self):

        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform('map', 'odom', now)
            
        except TransformException:
            self.get_logger().info(f'Could not listen to transform. Defaulting robot position.')
            return

        self.robot_position = [transform.transform.translation.x,
                            transform.transform.translation.y]
        
        self.robot_pose = self.create_pose(x= transform.transform.translation.x,
                                           y= transform.transform.translation.y,
                                           w= transform.transform.rotation.w,
                                           z= transform.transform.rotation.z)
        
    def publish_overlay_msg(self):

        string_msg = String()

        msg = f'''Pose: [{self.robot_pose.pose.position.x}, {self.robot_pose.pose.position.y}, {self.robot_pose.pose.position.z}] \n
                State: {self.current_state} \n'''
        
        string_msg.data = msg
        self.overlay_publisher.publish(string_msg)


    def navigator_timer_callback(self):


        if self.current_state == "Stand-by":

            self.reset_goal_arrays()

            if ((self.joy_buttons[BUTTON_TRIANGLE] != self.joy_buttons_last[BUTTON_TRIANGLE]) 
                and self.joy_buttons[BUTTON_TRIANGLE] == 1):

                self.current_state = self.waypoint_manager_states[1]
                self.get_logger().info("Opening recording. Changing to {0} state.\n".format(self.current_state))


        elif self.current_state == "Recording":

            if ((self.joy_buttons[BUTTON_TRIANGLE] != self.joy_buttons_last[BUTTON_TRIANGLE]) 
                and self.joy_buttons[BUTTON_TRIANGLE] == 1):

                if len(self.goal_waypoints) == 0:
                    self.create_default_exploration()
                    self.get_logger().info("Waypoint[] empty. Creating default exploration sequence.\n")

                self.current_state = self.waypoint_manager_states[2]
                self.get_logger().info("Closing recording. Changing to {0} state.\n".format(self.current_state))


            if ((self.joy_buttons[BUTTON_SQUARE] != self.joy_buttons_last[BUTTON_SQUARE]) 
                and self.joy_buttons[BUTTON_SQUARE] == 1):

                self.get_robot_pose()

                self.goal_poses.append(self.robot_pose)
                self.goal_waypoints.append(self.robot_position)

                self.get_logger().info("Saved {0} Waypoint.".format(self.robot_position))


        elif self.current_state == 'Setup':
            
            self.nav_start = self.navigator.get_clock().now()

            self.current_goal_pose = self.goal_poses[self.goal_index]
            self.current_goal_waypoint = self.goal_poses[self.goal_index]

            self.navigator.goToPose(self.current_goal_pose)

            self.current_state = self.waypoint_manager_states[3]

            self.get_logger().info("Set goal waypoint to {0}.\n".format(self.current_goal_waypoint))
            self.get_logger().info("Changing to {0} state.\n".format(self.current_state))

        
        elif self.current_state == 'Navigating':
            
            if self.navigator.isTaskComplete() == False:
                feedback = self.navigator.getFeedback()

                self.get_logger().info("Navigating to {0} waypoint. Estimated time of arrival {1:.0f}.\n".format(
                    self.current_goal_waypoint, Duration.from_msg(feedback.estimated_time_remaining).nanoseconds))

            else:
                self.current_state = self.waypoint_manager_states[4]
                self.get_logger().info("Task finished. Changing to {0} state.\n".format(self.current_state))


            if ((self.joy_buttons[BUTTON_TRIANGLE] != self.joy_buttons_last[BUTTON_TRIANGLE]) 
                and self.joy_buttons[BUTTON_TRIANGLE] == 1):

                self.navigator.cancelTask()
                self.current_state = self.waypoint_manager_states[4]
                self.get_logger().info("Canceling navigation task. Changing to {0} state.\n".format(self.current_state))

                
        elif self.current_state == 'End':

            result = self.navigator.getResult()

            if result == TaskResult.SUCCEEDED:

                self.goal_index += 1

                try:
                    self.current_goal_waypoint = self.goal_poses[self.goal_index]

                    self.current_state = self.waypoint_manager_states[2]
                    self.get_logger().info("{0}/{1} goals succeeded. Changing to {2} state.\n".format(
                        self.goal_index + 1, len(self.goal_waypoints), self.current_state))

                except IndexError:
                    self.current_state = self.waypoint_manager_states[0]
                    self.get_logger().info("All goals succeeded. Starting next navigation. to {0} state.\n".format(self.current_state))

            elif result == TaskResult.CANCELED:
                self.current_state = self.waypoint_manager_states[0]
                self.get_logger().info("Goal canceled. Changing to {0} state.\n".format(self.current_state))

            elif result == TaskResult.FAILED:
                self.current_state = self.waypoint_manager_states[0]
                self.get_logger().info("Goal failed. Changing to {0} state.\n".format(self.current_state))

            else:
                self.current_state = self.waypoint_manager_states[0]
                self.get_logger().info("Invalid return status. Changing to {0} state.\n".format(self.current_state))

        self.get_robot_pose()
        self.publish_overlay_msg()

        self.joy_buttons_last = self.joy_buttons


    def waypoint_publisher_timer_callback(self):

        msg_array = np.array(self.goal_waypoints)

        msg = Float64MultiArray()
        msg.data = msg_array.reshape(int(msg_array.size)).tolist()

        self.waypoint_publisher.publish(msg)


    def joy_callback(self, msg):
        self.joy_buttons = msg.buttons


    def create_pose(self, x,y,w,z):

        pose = PoseStamped()
        pose.header.frame_id = '/map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = w
        pose.pose.orientation.z = z

        waypoint = [x,y]

        return pose, waypoint
    
    def create_default_exploration(self):
        
        gp1, gw1 = self.create_pose(x= 2.0, y= 0.0, w= 1.0, z= 0.0)
        self.goal_poses.append(gp1)
        self.goal_waypoints.append(gw1)

        '''

        gp2, gw2 = self.create_pose(x= 2.0, y= 2.0, w= 0.0, z= 1.0)
        self.goal_poses.append(gp2)
        self.goal_waypoints.append(gw2)
        
        gp3, gw3 = self.create_pose(x= 0.0, y= 2.0, w= 0.0, z= 1.0)
        self.goal_poses.append(gp3)
        self.goal_waypoints.append(gw3)

        gp4, gw4 = self.create_pose(x= -2.0, y= 2.0, w= 0.0, z= 1.0)
        self.goal_poses.append(gp4)
        self.goal_waypoints.append(gw4)

        gp5, gw5 = self.create_pose(x= -2.0, y= 0.0, w= 1.0, z= 0.0)
        self.goal_poses.append(gp5)
        self.goal_waypoints.append(gw5)   
        '''     


def main():
    rclpy.init()
    
    manager = WaypointManager()

    rclpy.spin(manager)

    manager.navigator.lifecycleShutdown()
    manager.destroy_node()

    exit(0)

if __name__ == '__main__':
    main()