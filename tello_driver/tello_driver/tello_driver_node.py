import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from tello_msg.action import TelloCommand
from tello_msg.msg import TelloStatus
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CameraInfo

import yaml
import socket
import threading
import h264decoder
import numpy as np
from cv_bridge import CvBridge

class TelloDriverNode(Node):
    def __init__(self):
        super().__init__('tello_driver_node')

        # Parameters
        self.declare_parameter('tello_ip', '192.168.10.1')
        self.declare_parameter('response_receive_port', 8889)
        self.declare_parameter('state_receive_port', 8890)
        self.declare_parameter('video_receive_port', 11111)

        self.tello_ip = str(self.get_parameter('tello_ip').value)

        # Action server for commands (takeoff, land, etc)
        self.command_server = ActionServer(self, TelloCommand, 'command', self.cb_command)
        
        # Status receive thread related
        self.status_thread = threading.Thread(target=self.status_receive_thread)
        self.status_thread.daemon = True
        self.status_pub_timer = self.create_timer(0.1, self.status_publish)
        self.status_latest = None

        # Video receive thread related
        self.frames_thread = threading.Thread(target=self.frames_receive_thread)
        self.frames_thread.daemon = True
        self.frames_pub_timer = self.create_timer(0.03, self.frames_publish)
        self.frames_latest = None
        self.frames_raw_data = b''
        self.decoder = h264decoder.H264Decoder()
        self.bridge = CvBridge()

        # Camera info publisher related
        self.fname_cam_down = '/home/arthur/tello_ws/src/ros2_tello_driver/tello_bringup/config/camera_info_down_v2.yaml'
        self.fname_cam_front = '/home/arthur/tello_ws/src/ros2_tello_driver/tello_bringup/config/camera_info_front_v2.yaml'
        self.front_camera_info_msg = None
        self.down_camera_info_msg = None
        
        self.load_camera_info()
        self.camera_info_pub_timer = self.create_timer(0.1, self.camera_info_publish)

        # Publishers and subscribers
        self.status_pub = self.create_publisher(TelloStatus, 'status', 1)
        self.frames_pub = self.create_publisher(Image, 'image_raw', 1)
        self.camera_info_pub = self.create_publisher(CameraInfo, 'camera_info', 1)
        self.control_sub = self.create_subscription(Twist, 'cmd_vel', self.cb_control, 1)

        # Socket related
        self.control_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.status_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.frames_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.control_socket.bind(('0.0.0.0', int(self.get_parameter('response_receive_port').value)))
        self.status_socket.bind(('0.0.0.0', int(self.get_parameter('state_receive_port').value)))
        self.frames_socket.bind(('0.0.0.0', int(self.get_parameter('video_receive_port').value)))

        self.control_socket.settimeout(10)
        self.status_socket.settimeout(10)
        self.frames_socket.settimeout(10)

        r1 = self.send_command("command", True)
        r2 = self.send_command(f"port {int(self.get_parameter('state_receive_port').value)} {int(self.get_parameter('video_receive_port').value)}", True)
        r3 = self.send_command("streamon", True)

        if (r1 and r2 and r3):
            print("Connected to drone and setup complete")
            self.status_thread.start()
            self.frames_thread.start()
        else:
            print("Error connecting to drone, shutting down node")

    def load_camera_info(self):
        try:

            with open(self.fname_cam_front, "r") as file_handle:
                calib_data = yaml.safe_load(file_handle)

            # Parse
            self.front_camera_info_msg = CameraInfo()
            self.front_camera_info_msg.header.frame_id = 'camera'
            self.front_camera_info_msg.width = calib_data["image_width"]
            self.front_camera_info_msg.height = calib_data["image_height"]
            self.front_camera_info_msg.k = calib_data["camera_matrix"]["data"]
            self.front_camera_info_msg.d = calib_data["distortion_coefficients"]["data"]
            self.front_camera_info_msg.r = calib_data["rectification_matrix"]["data"]
            self.front_camera_info_msg.p = calib_data["projection_matrix"]["data"]
            self.front_camera_info_msg.distortion_model = calib_data["distortion_model"]

        except Exception as e:
            print(f"Error: {e}")

    def status_receive_thread(self):
        while True:
            try:
                data, address = self.status_socket.recvfrom(1024)
                self.status_latest = data.decode()

            except Exception as e:
                print(f"Error: {e}")

    def frames_receive_thread(self):
        while True:
            try:
                data, address = self.frames_socket.recvfrom(2048)

                self.frames_raw_data += data
                # end of frame
                if len(data) != 1460:
                    for frame in self.frames_decode(self.frames_raw_data):
                        self.frames_latest = frame
                    self.frames_raw_data = b''

            except Exception as e:
                print(f"Error: {e}")

    def status_publish(self):
        if (self.status_latest is not None):
            msg = TelloStatus()
            msg.acceleration.x = self.status_decode("agx", self.status_latest)
            msg.acceleration.y = self.status_decode("agy", self.status_latest)
            msg.acceleration.z = self.status_decode("agz", self.status_latest)
            msg.speed.x = self.status_decode("vgx", self.status_latest)
            msg.speed.y = self.status_decode("vgy", self.status_latest)
            msg.speed.z = self.status_decode("vgz", self.status_latest)
            msg.orientation.x = self.status_decode("pitch", self.status_latest)
            msg.orientation.y = self.status_decode("roll", self.status_latest)
            msg.orientation.z = self.status_decode("yaw", self.status_latest)
            msg.battery = self.status_decode("bat", self.status_latest)
            msg.distance_tof = self.status_decode("tof", self.status_latest)
            self.status_pub.publish(msg)

    def frames_publish(self):
        if (self.frames_latest is not None):
            msg = self.bridge.cv2_to_imgmsg(np.array(self.frames_latest), 'rgb8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "drone"
            self.frames_pub.publish(msg)

    def camera_info_publish(self):
        if (self.front_camera_info_msg is not None):
            self.front_camera_info_msg.header.stamp = self.get_clock().now().to_msg()
            self.camera_info_pub.publish(self.front_camera_info_msg)

    def status_decode(self, sub_str, str):
        start_idx = str.find(sub_str)
        if start_idx != -1:
            end_idx = str.find(";", start_idx)
            if end_idx != -1:
                return float(str[start_idx+len(sub_str)+1 : end_idx])
        else:
            return None

    def frames_decode(self, packet_data):
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
    
    def cb_control(self, msg):
        self.send_command(f"rc {int(msg.linear.x*100)} {int(msg.linear.y*100)} {int(msg.linear.z*100)} {int(msg.angular.z*100)}", False)


def main(args=None):
    rclpy.init(args=args)

    tello_driver_node = TelloDriverNode()

    rclpy.spin(tello_driver_node)

    tello_driver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
