import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare argument for enabling/disabling recording
    record_enabled = LaunchConfiguration('record', default='true')

    return LaunchDescription([
        # Launch argument for enabling/disabling recording
        DeclareLaunchArgument(
            'record',
            default_value='true',
            description='Enable or disable bag recording'
        ),

        # Publisher node
        Node(
            package='beginner_tutorials',
            executable='publisher_node',
            name='publisher_node',
            parameters=[{'publish_frequency': 2.0}],
            output='screen'
        ),

        # Subscriber node
        Node(
            package='beginner_tutorials',
            executable='subscriber_node',
            name='subscriber_node',
            output='screen'
        ),

        # Conditional recording
        ExecuteProcess(
            condition=IfCondition(record_enabled),
            cmd=[
                'ros2', 'bag', 'record', '-a', '-o', 'rosbag_output'
            ],
            output='screen'
        ),
    ])
