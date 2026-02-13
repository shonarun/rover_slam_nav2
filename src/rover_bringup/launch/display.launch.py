from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare(package='sam_bot_description').find('sam_bot_description')
    default_model_path = os.path.join(pkg_share, 'src', 'description', 'sam_bot_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'config.rviz')

    return LaunchDescription([
        
    ])