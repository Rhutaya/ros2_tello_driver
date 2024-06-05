import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from tello_msg.action import TelloCommand
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, Image

class NavActionClient(Node):

    def __init__(self):
        super().__init__('drone_nav_client')
        self._action_client = ActionClient(self, TelloCommand, 'command')
        self.control_pub_timer = self.create_timer(0.1, self.control_publish)
        
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.control_pub = self.create_publisher(Twist, 'cmd_vel', 1)

        self.prev_carre_state = 0
        self.pad = Pad()
        self.cmd = VelCommand()
        self.takeoff = 0
        self.downcam = 0
        
    def control_publish(self):
        # self.get_logger().info("%s" % self.cmd.lin_z) # header axes buttons
        if (self.cmd.lin_z is not None):
            msg = Twist()
            msg.linear.x = float(self.cmd.lin_x)
            msg.linear.y = float(self.cmd.lin_y)
            msg.linear.z = float(self.cmd.lin_z)
            msg.angular.z = float(self.cmd.rot_z)
            self.control_pub.publish(msg)

    def send_goal(self, order):
        goal_msg = TelloCommand.Goal()
        goal_msg.command = order

        self.get_logger().info("%s" % goal_msg)
        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)

    def joy_callback(self, msg):
        self.pad.header = {cle: valeur for cle, valeur in zip(self.pad.header.keys(), [msg.header.stamp.sec, msg.header.stamp.nanosec, msg.header.frame_id])}
        self.pad.axes = {cle: valeur for cle, valeur in zip(self.pad.axes.keys(), msg.axes)}
        self.pad.buttons = {cle: valeur for cle, valeur in zip(self.pad.buttons.keys(), msg.buttons)}
        # self.get_logger().info("%s" % self.pad.axes) # header axes buttons
        # action_client.send_goal(msg.data)
        
        if self.pad.buttons["triangle"] == 1:
            self.send_goal("takeoff")
            
        if self.pad.buttons["rond"] == 1:
            self.send_goal("land")
            
        if self.pad.buttons["carre"] == 1 and self.prev_carre_state == 0:
            if self.downcam == 0:
                self.send_goal("downvision 1")
                self.downcam = 1
            else:
                self.send_goal("downvision 0")
                self.downcam = 0
        
        self.prev_carre_state = self.pad.buttons["carre"]
         
        self.cmd.lin_y = self.pad.axes["L3V"]
        self.cmd.lin_x = - self.pad.axes["R3H"]
        self.cmd.lin_z = self.pad.axes["R3V"]
        self.cmd.rot_z = - self.pad.axes["L3H"]


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

    tello_nav = NavActionClient()

    try:
        rclpy.spin(tello_nav)
    except KeyboardInterrupt:
        pass

    tello_nav.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
