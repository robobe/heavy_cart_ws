import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    model_env= AppendEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        os.path.join(get_package_share_directory('heavy_cart_description'),
                     'models'))
    
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
    
    ld.add_action(worlds_env)
    ld.add_action(model_env)
    # ld.add_action(resource_env)
    ld.add_action(gazebo)
    return ld