import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap
from launch.substitutions import PathJoinSubstitution, TextSubstitution

def generate_launch_description():

    pioneer_pkg = get_package_share_directory('pioneer_navigation')
    robot_localization_file_path = os.path.join(pioneer_pkg, 'config/ekf.yaml')

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        parameters=[pioneer_pkg + '/config/mapping.yaml'],
        output='screen',
    )

    nav2_params = os.path.join(
        get_package_share_directory('pioneer_navigation'),
        'config',
        'nav2_params.yaml'
        )
    
    nav_include = GroupAction(
        actions=[

            SetRemap(src='/cmd_vel',dst='/cmd_vel_nav'),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('nav2_bringup'), 
                    'launch'),'/navigation_launch.py']),

                    launch_arguments={
                        'params_file' : nav2_params,
                        'parent_frame' : 'camera_link',
                        }.items(),
                    ),
        ]
    )
    

    return LaunchDescription([
        slam_node,
        nav_include,
    ])
