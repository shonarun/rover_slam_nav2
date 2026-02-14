from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # Declare argument for params file
    nav2_params_file = PathJoinSubstitution([
        FindPackageShare('rover_bringup'),
        'config',
        'nav2_params.yaml'
    ])
    declare_params = DeclareLaunchArgument(
        'nav2_params_file',
        default_value='nav2_params.yaml',
        description='Full path to the ROS2 parameters file to use for Nav2'
    )

    pkg_dir = get_package_share_directory('rover_bringup')
    static_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'static_tf.launch.py')
        )
    )

    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'display.launch.py')
        )
    )

    # Include Nav2 launch file
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ),
        launch_arguments={
            'params_file': nav2_params_file
        }.items()
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )

    costmap_markers_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d_markers',
        name='nav2_costmap_2d_markers',
        output='screen',
        remappings=[
            ('voxel_grid', '/local_costmap/voxel_grid'),
            ('visualization_marker', '/my_marker'),
        ]
    )
    
    teleop_keyboard_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='base_to_laser_tf',
        arguments=[
            '--ros-args', '-p', 'stamped:=true',
            '--remap', 'cmd_vel:=/demo/cmd_vel',
        ]
    )

    return LaunchDescription([
        static_tf_launch,
        declare_params,
        slam_launch,
        nav2_launch,
        costmap_markers_node,
        display_launch,
    ])
