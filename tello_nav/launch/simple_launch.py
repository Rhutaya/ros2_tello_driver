from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tello_nav',
            executable='tello_nav',
            name='tello_nav',
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
        ),
    ])