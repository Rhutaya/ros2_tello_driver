from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tello_driver',
            executable='tello_driver',
            name='drone',
            parameters=[{'tello_ip': '172.20.10.6', 'response_receive_port': 5000, 'state_receive_port': 5001, 'video_receive_port': 5002}]
        )
    ])