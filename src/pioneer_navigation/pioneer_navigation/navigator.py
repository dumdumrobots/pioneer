#! /usr/bin/env python3

import rclpy

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from rclpy.duration import Duration
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray

import numpy as np


class WaypointManager(Node):
    
    def __init__(self):

        super().__init__('waypoint_manager')

        self.goal_waypoints = []
        self.goal_poses = []
        
        gp1, gw1 = self.create_pose(x=1, y=0.0, w=0.0, z=1.0)
        self.goal_poses.append(gp1)
        self.goal_waypoints.append(gw1)

        '''
        gp2, gw2 = self.create_pose(x=1.5, y=1.5, w=0, z=1)
        self.goal_poses.append(gp1)
        self.goal_waypoints.append(gw1)

        gp3, gw3 = self.create_pose(x=0.0, y=1.5, w=0.707, z=-0.707)
        self.goal_poses.append(gp1)
        self.goal_waypoints.append(gw1)

        gp4, gw4 = self.create_pose(x=1.5, y=0.0, w=0.707, z=0.707)
        self.goal_poses.append(gp1)
        self.goal_waypoints.append(gw1)
        '''

        self.publish_timer = self.create_timer(0.1, self.publish_timer_callback)

        self.waypoint_publisher = self.create_publisher(Float64MultiArray, '/nav_waypoints', 10)

    def publish_timer_callback(self):

        msg_array = np.array(self.goal_waypoints)

        msg = Float64MultiArray()
        msg.data = msg_array.reshape(int(msg_array.size)).tolist()

        #self.get_logger().info("Publishing Waypoint Array: {0}".format(msg_array.reshape(int(msg_array.size)).tolist()))

        self.waypoint_publisher.publish(msg)


    def create_pose(self, x,y,w,z):

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = w
        pose.pose.orientation.z = z

        waypoint = [x,y]

        return pose, waypoint


def main():
    rclpy.init()

    navigator = BasicNavigator()
    manager = WaypointManager()

    initial_pose, initial_waypoint = manager.create_pose(x=0.0,y=0.0,w=1.0,z=0.0)
    navigator.setInitialPose(initial_pose)

    goal_waypoints = manager.goal_waypoints
    goal_poses = manager.goal_poses
    
    rclpy.spin_once(manager)
    
    #navigator.goThroughPoses(goal_poses)
    navigator.goToPose(goal_poses[0])

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