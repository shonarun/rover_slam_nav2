from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    # Declare argument for params file
    nav2_params_file = LaunchConfiguration('nav2_params_file')
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
    
    return LaunchDescription([
        static_tf_launch,
    ])
