
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
    
    config_topics = os.path.join(
        get_package_share_directory('pioneer_teleop'),
        'config',
        'topics.yaml'
        )
    
    config_locks = os.path.join(
        get_package_share_directory('pioneer_teleop'),
        'config',
        'locks.yaml'
        )
    

    teleop_twist_joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('teleop_twist_joy'), 
            'launch'),'/teleop-launch.py']),

            launch_arguments={
                'joy_config': 'ps4', 
                'joy_vel': 'cmd_vel_joy',
                'config_filepath' : config,
                }.items(),
            )
    
    
    twist_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('twist_mux'), 
            'launch'),'/twist_mux_launch.py']),

            launch_arguments={
                'config_topics': config_topics,
                'config_locks': config_locks,
                'cmd_vel_out': '/cmd_vel_out',
                }.items(),
            )
    

    switches_node = Node(
        package='pioneer_teleop',
        executable='switches',
        name='switches_node',
        output='screen',
    )

    return LaunchDescription([
        switches_node,
        teleop_twist_joy,
        #twist_mux_launch,
        
    ])

