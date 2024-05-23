
import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    description_pkg = get_package_share_directory('pioneer_description')
    file_subpath = 'robots/pioneer.urdf.xacro'

    xacro_file = os.path.join(description_pkg,file_subpath)
    robot_desc = xacro.process_file(xacro_file).toxml()

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz.'
    )

    # Get the parser plugin convert sdf to urdf using robot_description topic
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ],
        remappings=[
            ('/robot_description', '/robot_description'),
        ]
    )

    # Launch rviz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(description_pkg, 'rviz', 'pioneer.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )

    return LaunchDescription([
        rviz_launch_arg,
        robot_state_publisher,
        joint_state_pub,
        rviz,
    ])