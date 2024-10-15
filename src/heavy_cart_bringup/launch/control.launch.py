from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    # gazebo_ros_pkg_path = os.path.join(get_package_share_directory('gazebo_ros'))
    # controller_pkg_path = os.path.join(get_package_share_directory('your_controller_package'))
    
    return LaunchDescription([
        # Launch Gazebo Classic
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([gazebo_ros_pkg_path, '/launch/gazebo.launch.py'])
        # ),

        # Load controllers from controller_manager
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint1_position_controller', '--controller-manager', '/controller_manager'],
            output='screen'
        ),
    ])
