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

    switches_node = Node(
        package='pioneer_core',
        executable='switches',
        name='switches_node',
        output='screen',
    )
    

    return LaunchDescription([
        switches_node,
    ])