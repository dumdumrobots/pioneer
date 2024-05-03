
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('pioneer_teleop'),
        'config',
        'ps4.config.yaml'
        )
        
    teleop_twist_joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('teleop_twist_joy'), 'launch','teleop-launch.py')
            ),

            launch_arguments={
                'joy_config': 'ps4', 
                'config_filepath' : config
                }.items()
            )

    aria_node = Node(package='aria_bringup', 
                        executable='aria_bringup',
                        name='aria_node',
                        arguments=['-rp', '/dev/ttyUSB0']),
                        

    return LaunchDescription([
        teleop_twist_joy,
        aria_node,
    ])

