#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from visualization_msgs.msg import Marker


class WaypointManager(Node):
    
    def __init__(self, waypoints):

        self.publisher_dic = {}
        self.waypoint_dic = {}

        super().__init__('waypoint_manager')
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        for index, waypoint in enumerate(waypoints):

            name = 'marker_{0}'.format(index)
            
            self.publisher_dic[name] = self.create_publisher(Marker, name, 10)
            self.waypoint_dic[name] = self.create_marker_msg(index, waypoint[0], waypoint[1])
            self.get_logger().info("Marker {0} created.".format(name))


    def timer_callback(self):
        for name in self.waypoint_dic:
            self.publisher_dic[name].publish(self.waypoint_dic[name])

    def create_marker_msg(self, id, x, y):
        
        name = 'marker_{0}'.format(id)

        marker = Marker()
        marker.header.frame_id = '/map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = marker.CUBE
        marker.id = id
        marker.action = marker.ADD

        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.3

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.15
        
        return marker


def create_pose(navigator, x,y,w,z):

    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.w = w
    pose.pose.orientation.z = z

    waypoint = [x,y]

    return pose, waypoint


def main():
    rclpy.init()

    navigator = BasicNavigator()

    initial_pose, initial_waypoint = create_pose(navigator=navigator,x=0.0,y=0.0,w=1.0,z=0.0)
    navigator.setInitialPose(initial_pose)

    # set our demo's goal poses
    goal_waypoints = []
    goal_poses = []

    goal_pose1, goal_waypoint1 = create_pose(navigator=navigator,x=1.5,y=0.0,w=0.707,z=0.707)
    goal_poses.append(goal_pose1)
    goal_waypoints.append(goal_waypoint1)

    '''
    goal_pose2, goal_waypoint2 = create_pose(navigator=navigator,x=1.5,y=1.5,w=0,z=1)
    goal_poses.append(goal_pose2)
    goal_waypoints.append(goal_waypoint2)

    goal_pose3, goal_waypoint3 = create_pose(navigator=navigator,x=0.0,y=1.5,w=0.707,z=-0.707)
    goal_poses.append(goal_pose3)
    goal_waypoints.append(goal_waypoint3)

    goal_poses.append(goal_pose1)
    goal_waypoints.append(goal_waypoint1)

    '''

    manager = WaypointManager(waypoints=goal_waypoints)
    rclpy.spin_once(manager)
    
    navigator.goThroughPoses(goal_poses)

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