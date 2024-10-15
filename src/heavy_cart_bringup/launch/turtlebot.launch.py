import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import xacro
from pathlib import Path

URDF = "turtlebot3_burger.xacro"
PKG_DESCRIPTION = "heavy_cart_description"

"""
load turtlebot urdf from xacro
run gazebo
spawn robot into gazebo

"""
def generate_launch_description():
    ld = LaunchDescription()

    pkg_description = get_package_share_directory(PKG_DESCRIPTION)
    xacro_file = Path(pkg_description).joinpath("urdf", URDF).as_posix()
    urdf = xacro.process_file(xacro_file).toxml()

    # model_env= AppendEnvironmentVariable(
    #     'GAZEBO_MODEL_PATH',
    #     os.path.join(get_package_share_directory('heavy_cart_description'),
    #                  'models'))
    
    # resource_env= AppendEnvironmentVariable(
    #     'GAZEBO_RESOURCE_PATH',
    #     os.path.join(get_package_share_directory('heavy_cart_description'),
    #                  'meshes'))
    
    worlds_env= AppendEnvironmentVariable(
        'GAZEBO_RESOURCE_PATH',
        os.path.join(get_package_share_directory('heavy_cart_gazebo'),
                     'worlds'))
    
    gazebo_launch_file = os.path.join(
        get_package_share_directory('gazebo_ros'),  
        'launch',
        'gazebo.launch.py' 
    )
    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_file),
            launch_arguments={'world': 'empty.world', 'verbose': 'true'}.items()
        )
    params = {"robot_description": urdf, "use_sim_time": True}
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "rrbot"
        ],
        output="screen",
    )

    ld.add_action(worlds_env)
    # ld.add_action(model_env)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(gazebo)
    ld.add_action(spawn_entity)
    return ld