import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution

def generate_launch_description():


    lidar_node = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        name='lidar_node',
        arguments=['/opt/ros/humble/share/sick_scan_xd/launch/sick_tim_7xx.launch']
        )
    
    
    aria_node = Node(
        package='aria_bringup', 
        executable='aria_bringup',
        name='aria_node',
        arguments=['-rp', '/dev/ttyUSB0']
        )
    

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('phidgets_spatial'), 
            'launch'),'/spatial-launch.py']),
            )
    

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('pioneer_teleop'), 
            'launch'),'/teleop.launch.py']),
            )
    
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('depthai_ros_driver'), 
            'launch'),'/camera.launch.py']),

            launch_arguments={
                'use_rviz': 'false',
                }.items(),
            )
        

    return LaunchDescription([
        imu_launch,
        teleop_launch,
        lidar_node,
        aria_node,
        camera_launch,
    ])
