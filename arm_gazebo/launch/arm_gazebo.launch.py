from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit


 
def generate_launch_description():
    
    declared_arguments = []

    
    gazebo_world = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('arm_gazebo'),
                                    'launch',
                                    'arm_world.launch.py'])]),
    )

    controllers = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('arm_control'),
                                    'launch',
                                    'arm_control.launch.py'])]),
    )
    
    nodes_to_start = [
        gazebo_world,
        controllers
    ]

    return LaunchDescription(nodes_to_start)
    
