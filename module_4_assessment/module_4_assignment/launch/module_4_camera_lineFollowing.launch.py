import os

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    bal_launch_file_dir = get_package_share_directory('bal_robot_urdf_description')  # Replace 'my_package' with your package name
    bal_launch_file = os.path.join(bal_launch_file_dir, 'launch', 'camera_linefollowing.launch.py')
    include_bal_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bal_launch_file)
    )

    return LaunchDescription([
        include_bal_launch

    ])
