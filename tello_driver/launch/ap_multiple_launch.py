from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tello_driver',
            executable='tello_driver',
            name='drone1',
            namespace='drone1',
            parameters=[{'tello_ip': '172.20.10.6', 'response_receive_port': 5000, 'state_receive_port': 5001, 'video_receive_port': 5002}]
        ),
        Node(
            package='tello_driver',
            executable='tello_driver',
            name='drone2',
            namespace='drone2',
            parameters=[{'tello_ip': '172.20.10.7', 'response_receive_port': 6000, 'state_receive_port': 6001, 'video_receive_port': 6002}]
        )
    ])