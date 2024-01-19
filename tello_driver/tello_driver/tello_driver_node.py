import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from tello_msg.action import TelloCommand
from tello_msg.msg import TelloStatus

import socket

class TelloDriverNode(Node):
    def __init__(self):
        super().__init__('tello_driver_node')

        self.declare_parameter('tello_ip', '172.20.10.6')
        self.declare_parameter('response_receive_port', 8889)
        self.declare_parameter('video_receive_port', 11111)
        self.declare_parameter('state_receive_port', 8890)

        self.action_server = ActionServer(self, TelloCommand, 'command', self.cb_command)
        self.receive_state_timer = self.create_timer(0.1, self.receive_state)

        self.tello_ip = str(self.get_parameter('tello_ip').value)

        self.control_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.state_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.control_socket.bind(('0.0.0.0', int(self.get_parameter('response_receive_port').value)))
        self.state_socket.bind(('0.0.0.0', int(self.get_parameter('state_receive_port').value)))

        self.control_socket.settimeout(10)
        self.state_socket.settimeout(10)

        r1 = self.send_command("command", True)
        r2 = self.send_command(f"port {int(self.get_parameter('state_receive_port').value)} {int(self.get_parameter('video_receive_port').value)}", True)
        r3 = self.send_command("streamon", True)

        if (r1 and r2 and r3):
            print("Connected to drone and setup complete")
        else:
            print("Error connecting to drone, shutting down node")

    def extract(self, sub_str, str):
        start_idx = str.find(sub_str)
        if start_idx != -1:
            end_idx = str.find(";", start_idx)
            if end_idx != -1:
                print(str[start_idx+len(sub_str)+1 : end_idx])
                return 
        else:
            return None

    def pub_state(self, data):
        self.extract("agx", data)

    def receive_state(self):
        try:
            data, address = self.state_socket.recvfrom(1024)
            data = data.decode()
            self.pub_state(data)

        except Exception as e:
            print(f"Error: {e}")
        
    def send_command(self, msg, response):
        try:
            self.control_socket.sendto(msg.encode(), (self.tello_ip, 8889))
            if (response):
                data, server = self.control_socket.recvfrom(1024)
                response = data.decode()
                if ("ok" in response):
                    return True
                else:
                    return False
            else:
                return None

        except socket.timeout:
            print("Error: No response received within 10 seconds.")
            return False

        except Exception as e:
            print(f"Error: {e}")
            return False

    def cb_command(self, goal_handle):
        result = self.send_command(goal_handle.request.command, True)
        if result:
            goal_handle.succeed()

        action_result = TelloCommand.Result()
        action_result.result = result
        return action_result


def main(args=None):
    rclpy.init(args=args)

    tello_driver_node = TelloDriverNode()

    rclpy.spin(tello_driver_node)

    tello_driver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
