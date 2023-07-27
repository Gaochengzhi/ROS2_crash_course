import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DrawCircleNode(Node):
    def __init__(self, node_name="draw_cirlce") -> None:
        super().__init__("draw_circle")
        # 10 is queue size
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.timer = self.create_timer(0.5,self.send_velocity_callback)
        self.get_logger().info("draw circle node has been started")
    def send_velocity_callback(self):
        msg = Twist()
        # (base) ➜  ros2_ws ros2 interface show geometry_msgs/msg/Twist
        msg.linear.x = 3.0
        msg.angular.z = 3.0
        self.cmd_vel_pub_.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = DrawCircleNode()
    rclpy.spin(node)
    rclpy.shutdown()
