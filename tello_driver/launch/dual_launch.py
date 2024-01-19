from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tello_driver',
            namespace='drone1',
            executable='tello_driver',
            name='drone1',
            parameters=[{'tello_ip': '172.20.10.6', 'tello_video_port': 5000, 'tello_state_port': 5001}]
        )
    ])