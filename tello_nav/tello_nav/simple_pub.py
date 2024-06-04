import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class NavAction(Node):

    def __init__(self):
        super().__init__('drone_nav_client')
        self.control_pub_timer = self.create_timer(0.1, self.control_publish)
        
        self.subscription = self.create_subscription(Joy, 'joy', self.listener_callback, 10)
        self.control_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        
        self.pad = Pad()
        self.cmd = VelCommand()
        
    def control_publish(self):
        self.get_logger().info("%s" % self.cmd.lin_z) # header axes buttons
        if (self.cmd.lin_z is not None):
            msg = Twist()
            msg.linear.z = float(self.cmd.lin_z)
            self.control_pub.publish(msg)

    def listener_callback(self, msg):
        self.pad.axes = {cle: valeur for cle, valeur in zip(self.pad.axes.keys(), msg.axes)}
        self.cmd.lin_z = self.pad.axes["L3V"]
        # self.get_logger().info("%s" % self.pad.axes) # header axes buttonsh(msg)
        print(self.cmd.lin_z)
    
class Pad():
    def __init__(self):
        self.header = {"sec":0.0, "nanosec":0.0, "frame_id":0.0}
        self.axes = {"L3H":0.0, "L3V":0.0, "L2":0.0, "R3H":0.0, "R3V":0.0, "R2":0.0, "CrossH":0.0, "CrossV":0.0}   
        self.buttons = {"croix":0.0, "rond":0.0, "triangle":0.0, "carre":0.0, "L1":0.0, "R1":0.0, "L2":0.0, "R2":0.0, "share":0.0, "start":0.0, "PS":0.0, "L3":0.0, "R3":0.0}
   
class VelCommand():
    def __init__(self):
        self.lin_x = None
        self.lin_y = None
        self.lin_z = None
        self.rot_z = None
    
def main(args=None):
    rclpy.init(args=args)

    tello_nav = NavAction()

    rclpy.spin(tello_nav)

    tello_nav.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
