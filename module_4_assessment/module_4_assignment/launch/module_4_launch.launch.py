import os

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    bal_launch_file_dir = get_package_share_directory('bal_robot_urdf_description')  # Replace 'my_package' with your package name
    bal_launch_file = os.path.join(bal_launch_file_dir, 'launch', 'gazebo.launch.py')
    include_bal_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bal_launch_file)
    )

    lidar_sensing_node = Node(
        package='module_4_assignment',  # Replace with your package name
        executable='task3',  # Replace with your node executable name
        name='lidar_sensing_node',
        output='screen'
        )

    wall_following_node = Node(
        package='module_4_assignment',  # Replace with your package name
        executable='wall_following',  # Replace with your node executable name
        name='wall_following_node',
        output='screen'
        )
    imu_data_node = Node(
        package='module_4_assignment',  # Replace with your package name
        executable='imu_data_process',  # Replace with your node executable name
        name='imu_data_node',
        output='screen'
        )
    return LaunchDescription([
        lidar_sensing_node, 
        include_bal_launch,
        wall_following_node,
        imu_data_node
    ])
