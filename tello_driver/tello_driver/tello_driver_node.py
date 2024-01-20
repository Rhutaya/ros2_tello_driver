import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from tello_msg.action import TelloCommand
from tello_msg.msg import TelloStatus
from sensor_msgs.msg import Image

import socket
import h264decoder
import numpy as np
from cv_bridge import CvBridge

class TelloDriverNode(Node):
    def __init__(self):
        super().__init__('tello_driver_node')

        self.declare_parameter('tello_ip', '172.20.10.6')
        self.declare_parameter('response_receive_port', 8889)
        self.declare_parameter('state_receive_port', 8890)
        self.declare_parameter('video_receive_port', 11111)

        self.action_server = ActionServer(self, TelloCommand, 'command', self.cb_command)
        self.receive_state_timer = self.create_timer(0.1, self.receive_state)
        self.receive_video_timer = self.create_timer(0.0001, self.receive_video)
        
        self.pub_status = self.create_publisher(TelloStatus, 'status', 1)
        self.pub_image = self.create_publisher(Image, 'image_raw', 1)

        self.tello_ip = str(self.get_parameter('tello_ip').value)

        self.control_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.state_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.video_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.control_socket.bind(('0.0.0.0', int(self.get_parameter('response_receive_port').value)))
        self.state_socket.bind(('0.0.0.0', int(self.get_parameter('state_receive_port').value)))
        self.video_socket.bind(('0.0.0.0', int(self.get_parameter('video_receive_port').value)))

        self.control_socket.settimeout(10)
        self.state_socket.settimeout(10)
        self.video_socket.settimeout(10)

        self.video_data = b''
        self.decoder = h264decoder.H264Decoder()
        self.bridge = CvBridge()

        r1 = self.send_command("command", True)
        r2 = self.send_command(f"port {int(self.get_parameter('state_receive_port').value)} {int(self.get_parameter('video_receive_port').value)}", True)
        r3 = self.send_command("streamon", True)

        if (r1 and r2 and r3):
            print("Connected to drone and setup complete")
        else:
            print("Error connecting to drone, shutting down node")

    def extract_state(self, sub_str, str):
        start_idx = str.find(sub_str)
        if start_idx != -1:
            end_idx = str.find(";", start_idx)
            if end_idx != -1:
                return float(str[start_idx+len(sub_str)+1 : end_idx])
        else:
            return None

    def receive_state(self):
        try:
            data, address = self.state_socket.recvfrom(1024)
            data = data.decode()
            msg = TelloStatus()
            msg.acceleration.x = self.extract_state("agx", data)
            msg.acceleration.y = self.extract_state("agy", data)
            msg.acceleration.z = self.extract_state("agz", data)
            msg.speed.x = self.extract_state("vgx", data)
            msg.speed.y = self.extract_state("vgy", data)
            msg.speed.z = self.extract_state("vgz", data)
            msg.orientation.x = self.extract_state("pitch", data)
            msg.orientation.y = self.extract_state("roll", data)
            msg.orientation.z = self.extract_state("yaw", data)
            msg.battery = self.extract_state("bat", data)
            msg.distance_tof = self.extract_state("tof", data)
            self.pub_status.publish(msg)

        except Exception as e:
            print(f"Error: {e}")

    def h264_decode(self, packet_data):
        res_frame_list = []
        frames = self.decoder.decode(packet_data)
        for framedata in frames:
            (frame, w, h, ls) = framedata
            if frame is not None:

                frame = np.fromstring(frame, dtype=np.ubyte, count=len(frame), sep='')
                frame = (frame.reshape((h, int(ls / 3), 3)))
                frame = frame[:, :w, :]
                res_frame_list.append(frame)

        return res_frame_list

    def receive_video(self):
        try:
            data, address = self.video_socket.recvfrom(2048)

            self.video_data += data
            # end of frame
            if len(data) != 1460:
                for frame in self.h264_decode(self.video_data):
                    msg = self.bridge.cv2_to_imgmsg(np.array(frame), 'rgb8')
                    msg.header.frame_id = "drone"
                    self.pub_image.publish(msg)
                self.video_data = b''

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
