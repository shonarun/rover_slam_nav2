from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    rover_bringup_pkg_share = get_package_share_directory(package='rover_bringup').find('rover_bringup')
    rover_description_pkg_share = get_package_share_directory(package='rover_description').find('rover_description')
    default_model_path = os.path.join(rover_description_pkg_share, 'urdf', 'rover.sdf')
    default_rviz_config_path = os.path.join(rover_bringup_pkg_share, 'rviz', 'config.rviz')
    bridge_config_path = os.path.join(rover_bringup_pkg_share, 'config', 'bridge_config.yaml')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot model file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
        robot_state_publisher_node,
        rviz_node

    ])