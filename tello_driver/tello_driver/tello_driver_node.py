import rclpy
from rclpy.node import Node

from djitellopy import Tello

from std_msgs.msg import Empty, String
from geometry_msgs.msg import Twist


class TelloDriverNode(Node):

    def __init__(self):
        super().__init__('tello_driver_node')

        self.declare_parameter('tello_timeout', 10.0)
        self.declare_parameter('tello_ip', '192.168.10.1')

        self.connect_timeout = float(self.get_parameter('tello_timeout').value)
        self.tello_ip = str(self.get_parameter('tello_ip').value)

        Tello.TELLO_IP = self.tello_ip
        Tello.RESPONSE_TIMEOUT = int(self.connect_timeout)
        
        self.tello = Tello()
        self.tello.connect()

        self.get_logger().info('Tello: Connected to drone')

        self.sub_emergency_ = self.create_subscription(Empty, 'emergency', self.cb_emergency_, 1)
        self.sub_takeoff_ = self.create_subscription(Empty, 'takeoff', self.cb_takeoff_, 1)
        self.sub_land_ = self.create_subscription(Empty, 'land', self.cb_land_, 1)
        self.sub_control_ = self.create_subscription(Twist, 'cm_vel', self.cb_control_, 1)
        self.sub_flip_ = self.create_subscription(String, 'flip', self.cb_flip_, 1)

    def tello_status_thread_(self):
        pass

    def tello_video_thread_(self):
        pass

    def cb_emergency_(self, msg):
        self.tello.emergency()

    def cb_takeoff_(self, msg):
        self.tello.takeoff()

    def cb_land_(self, msg):
        self.tello.land()

    def cb_control_(self, msg):
        self.tello.send_rc_control(int(msg.linear.x), int(msg.linear.y), int(msg.linear.z), int(msg.angular.z))

    def cb_flip_(self, msg):
        self.tello.flip(msg.data)

def main(args=None):
    rclpy.init(args=args)

    tello_driver_node = TelloDriverNode()

    rclpy.spin(tello_driver_node)

    tello_driver_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
