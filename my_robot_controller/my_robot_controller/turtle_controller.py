import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import random



class PoseSubscriberNode(Node):
    def __init__(self) -> None:
        super().__init__("turtle_controller")
        self.cmd_vel_publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        # 10 is queue size
        self.pose_subscriber = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback,10)

    def pose_callback(self, pos:Pose):
        # (base) ➜  ros2_ws ros2 interface show geometry_msgs/msg/Twist
        cmd = Twist()
        # self.get_logger().info(str(pos))
        if pos.x > 9.0 or pos.x < 1.0 or pos.y > 9.0 or pos.y < 1.0:
            cmd.linear.x = 2.0*random.randint(1,4)
            cmd.angular.z = (random.random()+0.3)*10
        else:
            cmd.linear.x = 5.0
            cmd.angular.z = random.random()*3*((-1)**random.randint(1,99))
        self.cmd_vel_publisher.publish(cmd)
def main(args=None):
    rclpy.init(args=args)
    node = PoseSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()