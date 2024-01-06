from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'obstacle',
            default_value='0.3',
            description='Obstacle threshold value'
        ),
        DeclareLaunchArgument(
            'degrees',
            default_value='-90.0',
            description='Rotation degrees'
        ),
        DeclareLaunchArgument(
            'final_approach',
            default_value='false',
            description='Flag to start final approach'
        ),
        Node(
            package='attach_shelf',
            executable='pre_approach_node',
            name='pre_approach_node',
            output='screen',
            parameters=[
                {'obstacle': LaunchConfiguration('obstacle')},
                {'degrees': LaunchConfiguration('degrees')},
                {'final_approach': LaunchConfiguration('final_approach')}
            ]
        )
    ])
