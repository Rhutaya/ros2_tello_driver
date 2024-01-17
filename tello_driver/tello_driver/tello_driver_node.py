import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

import time
import threading
import numpy as np
from djitellopy import Tello
from cv_bridge import CvBridge

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from tello_msg.msg import TelloStatus

from tello_msg.action import TelloCommand

class TelloDriverNode(Node):
    def __init__(self):
        super().__init__('tello_driver_node')

        self.tello = Tello()
        self.tello.connect()
        self.tello.streamon()

        self.get_logger().info('Tello: Connected to drone')

        self.bridge_ = CvBridge()

        self.action_server_ = ActionServer(self, TelloCommand, 'command', self.cb_command_)
        
        self.sub_control_ = self.create_subscription(Twist, 'cmd_vel', self.cb_control_, 1)
        self.pub_image_ = self.create_publisher(Image, 'image_raw', 1)
        self.pub_status_ = self.create_publisher(TelloStatus, 'status', 1)

        image_thread_ = threading.Thread(target=self.tello_video_thread_)
        status_thread_ = threading.Thread(target=self.tello_status_thread_)
        image_thread_.start()
        status_thread_.start()

    def tello_status_thread_(self, rate=1.0/10.0):
        while True:
            msg = TelloStatus()
            msg.acceleration.x = self.tello.get_acceleration_x()
            msg.acceleration.y = self.tello.get_acceleration_y()
            msg.acceleration.z = self.tello.get_acceleration_z()
            msg.speed.x = float(self.tello.get_speed_x())
            msg.speed.y = float(self.tello.get_speed_y())
            msg.speed.z = float(self.tello.get_speed_z())
            msg.pitch = self.tello.get_pitch()
            msg.roll = self.tello.get_roll()
            msg.yaw = self.tello.get_yaw()
            msg.barometer = int(self.tello.get_barometer())
            msg.distance_tof = self.tello.get_distance_tof()
            msg.fligth_time = self.tello.get_flight_time()
            msg.battery = self.tello.get_battery()
            msg.highest_temperature = self.tello.get_highest_temperature()
            msg.lowest_temperature = self.tello.get_lowest_temperature()
            msg.temperature = self.tello.get_temperature()
            self.pub_status_.publish(msg)
            
            time.sleep(rate)

    def tello_video_thread_(self, rate=1.0/30.0):
        frame_read = self.tello.get_frame_read()

        while True:
            frame = frame_read.frame

            msg = self.bridge_.cv2_to_imgmsg(np.array(frame), 'rgb8')
            msg.header.frame_id = "drone"
            self.pub_image_.publish(msg)

            time.sleep(rate)

    def cb_command_(self, goal_handle):
        self.get_logger().info(f'Executing command...{goal_handle.request.command}')

        result = self.tello.send_control_command(goal_handle.request.command)

        if result:
            goal_handle.succeed()

        result = TelloCommand.Result()
        return result

    def cb_control_(self, msg):
        self.tello.send_rc_control(int(msg.linear.x*100), int(msg.linear.y*100), int(msg.linear.z*100), int(msg.angular.z*100))

def main(args=None):
    rclpy.init(args=args)

    tello_driver_node = TelloDriverNode()

    rclpy.spin(tello_driver_node)

    tello_driver_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
