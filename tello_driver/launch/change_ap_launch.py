from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tello_driver',
            executable='tello_driver',
            name='drone',
        ),
        Node(
            package='tello_driver',
            executable='connect_wifi',
            name='connect_wifi',
            parameters=[{'ssid': 'iPhone', 'password': 'helloworld'}]
        ),
    ])