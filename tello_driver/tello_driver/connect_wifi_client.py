import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from tello_msg.action import TelloCommand


class ConnectWifiClient(Node):
    def __init__(self):
        super().__init__('connect_wifi_client')
        self.command_client = ActionClient(self, TelloCommand, 'command')

    def send_goal(self, command):
        command_msg = TelloCommand.Goal()
        command_msg.command = command

        self.command_client.wait_for_server()

        self._send_goal_future = self.command_client.send_goal_async(command_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result:
            self.get_logger().info('Success')
        else:
            self.get_logger().info('Failure')

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    action_client = ConnectWifiClient()

    action_client.declare_parameter('ssid', 'iPhone')
    action_client.declare_parameter('password', 'helloworld')

    action_client.send_goal(f"ap {str(action_client.get_parameter('ssid').value)} {str(action_client.get_parameter('password').value)}")

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()