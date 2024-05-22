#! /usr/bin/env python3

import rclpy
import numpy as np

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from rclpy.duration import Duration
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener 

BUTTON_TRIANGLE = 2
BUTTON_SQUARE = 3

class WaypointManager(Node):
    
    def __init__(self):

        super().__init__('waypoint_manager')

        self.joy_buttons = [0,0,0,0,0,0,0,0,0]
        self.joy_buttons_last = [0,0,0,0,0,0,0,0,0]

        self.goal_waypoints = []
        self.goal_poses = []

        self.robot_pose, self.robot_position = self.create_pose(x= 0.0,y= 0.0,w= 1.0,z= 0.0)

        self.joy_subscriber = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.publisher_timer = self.create_timer(0.1, self.publisher_timer_callback)
        
        self.recording_timer = self.create_timer(0.1, self.recording_timer_callback)

        self.waypoint_publisher = self.create_publisher(Float64MultiArray, '/nav_waypoints', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


    def recording_timer_callback(self):

        if ((self.joy_buttons[BUTTON_SQUARE] != self.joy_buttons_last[BUTTON_SQUARE]) 
            and self.joy_buttons[BUTTON_SQUARE] == 1):

            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform('map', 'odom', now)

            self.robot_position = [transform.transform.translation.x,
                                   transform.transform.translation.y]
            
            self.robot_pose = self.create_pose(x= transform.transform.translation.x,
                                               y= transform.transform.translation.y,
                                               w= transform.transform.rotation.w,
                                               z= transform.transform.rotation.z)

            self.goal_poses.append(self.robot_pose)
            self.goal_waypoints.append(self.robot_position)

            self.get_logger().info("Waypoint Added: {0}".format(self.robot_position))

        self.joy_buttons_last = self.joy_buttons


    def publisher_timer_callback(self):

        msg_array = np.array(self.goal_waypoints)

        msg = Float64MultiArray()
        msg.data = msg_array.reshape(int(msg_array.size)).tolist()

        self.waypoint_publisher.publish(msg)


    def joy_callback(self, msg):
        self.joy_buttons = msg.buttons


    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose
        self.robot_position = [msg.pose.pose.point.x, msg.pose.pose.point.y]


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


def main():
    rclpy.init()

    navigator = BasicNavigator()
    manager = WaypointManager()

    initial_pose = manager.robot_pose
    initial_waypoint = manager.robot_position
    navigator.setInitialPose(initial_pose)

    manager.get_logger().info("Recording Waypoints. Press BUTTON_TRIANGLE to close.")

    while manager.joy_buttons[BUTTON_TRIANGLE] == 0:
        rclpy.spin_once(manager)

    manager.get_logger().info("Closing recording.")

    if len(manager.goal_waypoints) == 0:
        manager.create_default_exploration()
        manager.get_logger().info("Waypoint[] empty. Creating default exploration sequence.")
        rclpy.spin_once(manager)

    else:
        manager.get_logger().info("Sending Waypoint[] saved: {0}".format(manager.goal_waypoints))

    goal_waypoints = manager.goal_waypoints
    goal_poses = manager.goal_poses

    navigator.followWaypoints(goal_poses)
    #navigator.goToPose(goal_poses[0])

    i = 0
    while not navigator.isTaskComplete():

        rclpy.spin_once(manager)

        i = i + 1
        feedback = navigator.getFeedback()

        if feedback and i % 5 == 0:
            print(
                'Estimated time of arrival: '
                + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                    / 1e9
                )
                + ' seconds.'
            )

            # Some navigation timeout to demo cancellation
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                navigator.cancelTask()

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    navigator.lifecycleShutdown()
    manager.destroy_node()

    exit(0)


if __name__ == '__main__':
    main()