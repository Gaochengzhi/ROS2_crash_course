import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv  import SetPen
import random
from functools import partial


class PoseSubscriberNode(Node):
    def __init__(self) -> None:
        super().__init__("turtle_controller")
        self.cmd_vel_publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        # 10 is queue size
        self.pose_subscriber = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback,10)

    def pose_callback(self, pos:Pose):
        # (base) ➜  ros2_ws ros2 interface show geometry_msgs/msg/Twist
        cmd = Twist()
        self.previous_x = 0
        # self.get_logger().info(str(pos))
        if pos.x > 9.0 or pos.x < 1.0 or pos.y > 9.0 or pos.y < 1.0:
            cmd.linear.x = 2.0*random.randint(1,4)
            cmd.angular.z = (random.random()+0.3)*10
        else:
            cmd.linear.x = 5.0
            cmd.angular.z = random.random()*3*((-1)**random.randint(1,99))
        self.cmd_vel_publisher.publish(cmd)

        if pos.x >5.5 and self.previous_x <=5.5:
            self.previous_x = pos.x
            self.get_logger().info("set color to red")
            self.call_set_pen_service(255,0,0,3,0)
        elif pos.x < 5.5 and self.previous_x>=5.5:
            self.previous_x = pos.x
            self.get_logger().info("set color to green")
            self.call_set_pen_service(0,255,0,0,3)

    def call_set_pen_service(self,r,g,b,width,off):
        client = self.create_client(SetPen,"/turtle1/set_pen")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("wait for the service")
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off
        future = client.call_async(request)
        future.add_done_callback(partial(self.call_set_pen_service))

    def callback_setPen(self,future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Serviece call failed: %r"%(e,))


def main(args=None):
    rclpy.init(args=args)
    node = PoseSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()