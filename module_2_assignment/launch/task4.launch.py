from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():

    turtlesim = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim'
    )
    spawn_turtle1 = ExecuteProcess(
        cmd= ['ros2','service','call','/spawn', 'turtlesim/srv/Spawn', "\"{x: 1.0, y: 1.0, theta: 1.57, name: 'turtle2'}\""],
        name='spawn_turtle1',
        shell=True
    )

    spawn_turtle2 = ExecuteProcess(
        cmd= ['ros2','service','call','/spawn', 'turtlesim/srv/Spawn', "\"{x: 3.0, y: 3.0, theta: 1.57, name: 'turtle3'}\""],
        name='spawn_turtle2',
        shell=True
    )

    spawn_turtle4 = ExecuteProcess(
        cmd= ['ros2','service','call','/spawn', 'turtlesim/srv/Spawn', "\"{x: 7.0, y: 7.0, theta: 1.57, name: 'turtle5'}\""],
        name='spawn_turtle4',
        shell=True
    )

    spawn_turtle5 = ExecuteProcess(
        cmd= ['ros2','service','call','/spawn', 'turtlesim/srv/Spawn', "\"{x: 9.0, y: 9.0, theta: 1.57, name: 'turtle6'}\""],
        name='spawn_turtle4',
        shell=True
    )

    turtle_drive3=Node(
        package='module_2_assignment',
        executable='task4',
        name='turtle_drive3',
        parameters=[
            {'cmd_vel_topic': '/turtle3/cmd_vel'},
            {'angularz': 0.1},
            {'linearx': 0.1}
        ]
    )

    turtle_drive1=Node(
        package='module_2_assignment',
        executable='task4',
        name='turtle_drive3',
        parameters=[
            {'cmd_vel_topic': '/turtle1/cmd_vel'},
            {'angularz': 0.1},
            {'linearx': 0.1}
        ]
    )
    turtle_drive5=Node(
        package='module_2_assignment',
        executable='task4',
        name='turtle_drive3',
        parameters=[
            {'cmd_vel_topic': '/turtle5/cmd_vel'},
            {'angularz': 0.1},
            {'linearx': 1.0}
        ]
    )
    turtle_drive2=Node(
        package='module_2_assignment',
        executable='task4',
        name='turtle_drive2',
        parameters=[
            {'cmd_vel_topic': '/turtle2/cmd_vel'},
            {'angularz': 0.1},
            {'linearx': 2.0}
        ]
    )
    
    turtle_drive6=Node(
        package='module_2_assignment',
        executable='task4',
        name='turtle_drive6',
        parameters=[
            {'cmd_vel_topic': '/turtle6/cmd_vel'},
            {'angularz': 0.1},
            {'linearx': 0.1}
        ]
    )
    return LaunchDescription([
        turtlesim,
        
        spawn_turtle1,
        spawn_turtle2,
        spawn_turtle4, 
        spawn_turtle5,
        
        turtle_drive2,
        turtle_drive3,
        turtle_drive1,
        turtle_drive5,
        turtle_drive6,
        
    ])
