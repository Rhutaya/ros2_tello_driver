import rclpy
from rclpy.node import Node

# from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray  
from aruco_msgs.msg import MarkerArray as ArucoMarkerArray

class MarkerBridge(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

        # Publisher of marker message
        self.marker_pub = self.create_publisher(MarkerArray, 'visualization_marker', 1)
        # self.timer = self.create_timer(1, self.marker_publish)

        # Subsciber of aruco message
        self.marker_sub = self.create_subscription(ArucoMarkerArray, 'marker_publisher/markers', self.aruco_callback, 10)
        self.marker_latest = None

        self.i = 0

    def aruco_callback(self, msg):
        # self.get_logger().info('Receive : "%s"' % msg)

        self.frame = msg.header.frame_id
        self.marker_latest = msg.markers
        self.marker_publish()

    def marker_publish(self):
        if (self.marker_latest is not None):
            msg = MarkerArray()

            for aruco_marker in self.marker_latest:
                # set general infos
                marker = Marker()
                marker.header.frame_id = self.frame
                marker.header.stamp = aruco_marker.header.stamp

                # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
                marker.type = 0
                marker.id = aruco_marker.id
                marker.action = 0

                # Set the scale of the msg
                marker.scale.x = 0.3 # x3 factor to slim the arrow
                marker.scale.y = 0.1
                marker.scale.z = 0.1

                # Set the color
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0

                # Set the lifetime
                marker.lifetime.sec = 0
                marker.lifetime.nanosec = 500 * 100000 # in ms

                # Set the pose of the msg
                marker.pose = aruco_marker.pose.pose # 2 level cuz 1st one include covariance

                # Add to bracket
                msg.markers.append(marker)

            self.marker_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    marker_bridge = MarkerBridge()

    rclpy.spin(marker_bridge)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    marker_bridge.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()