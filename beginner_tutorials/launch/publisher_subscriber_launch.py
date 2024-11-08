from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='beginner_tutorials',
            executable='publisher_node',
            name='publisher_node',
            parameters=[{'publish_frequency': 2.0}]
        ),
        Node(
            package='beginner_tutorials',
            executable='subscriber_node',
            name='subscriber_node'
        ),
    ])
