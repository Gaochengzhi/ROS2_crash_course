import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class PoseSubscriberNode(Node):
    def __init__(self) -> None:
        super().__init__("pose_subscriber")
        # 10 is queue size
        self.pose_subscriber = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback,10)
    def pose_callback(self, msg:Pose):
        # (base) ➜  ros2_ws ros2 interface show geometry_msgs/msg/Twist
        self.get_logger().info(str(msg))



def main(args=None):
    rclpy.init(args=args)
    node = PoseSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()
