from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    turtlesim = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim'
    )
    
    logerithmic = Node(
        package='module_2_assignment',  # Replace with your package name
        executable='test1',  # Replace with your node executable name
        name='drive_node',
        output='screen',
        parameters=[{'given_radius': 6}]  # Set the radius parameter here
        )
    
    return LaunchDescription([
        turtlesim,
        logerithmic,
    ])
