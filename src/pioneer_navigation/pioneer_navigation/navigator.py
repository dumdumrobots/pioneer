#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from marker import WaypointManager


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

    initial_pose = create_pose(navigator=navigator,x=0.0,y=0.0,w=1.0,z=0.0)

    navigator.setInitialPose(initial_pose)

    # set our demo's goal poses
    goal_waypoints = []
    goal_poses = []

    goal_pose1, goal_waypoint1 = create_pose(navigator=navigator,x=1.5,y=0.0,w=0.707,z=0.707)
    goal_poses.append(goal_pose1)
    goal_waypoints.append(goal_waypoint1)

    goal_pose2, goal_waypoint2 = create_pose(navigator=navigator,x=1.5,y=1.5,w=0,z=1)
    goal_poses.append(goal_pose2)
    goal_waypoints.append(goal_waypoint2)

    goal_pose3, goal_waypoint3 = create_pose(navigator=navigator,x=0.0,y=1.5,w=0.707,z=-0.707)
    goal_poses.append(goal_pose3)
    goal_waypoints.append(goal_waypoint3)

    goal_poses.append(goal_pose1)
    goal_waypoints.append(goal_waypoint1)

    manager = WaypointManager(waypoints=goal_waypoints)
    
    navigator.goThroughPoses(goal_poses)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
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