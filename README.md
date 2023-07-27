# ROS crash course

[https://www.youtube.com/watch?v=0aPbWsyENA8&list=PLLSegLrePWgJudpPUof4-nVFHGkB62Izy](https://www.youtube.com/watch?v=0aPbWsyENA8&list=PLLSegLrePWgJudpPUof4-nVFHGkB62Izy)

## ROS2 Humble

2022 released

### Graph view

```bash
rqt_graph
```

## Create your own ros package/node

### Use colcon [https://colcon.readthedocs.io/en/released/](https://colcon.readthedocs.io/en/released/)

Colcon (CMake/Catkin-based build tool for ROS 2)

> [colcon](http://colcon.readthedocs.io/) is a command line tool to improve the workflow of building, testing and using multiple software packages. It automates the process, handles the ordering and sets up the environment to use the packages.
> 
1. make your own workspace 
2. mkdir src
3. init a colcon project

```bash
colcon build

echo "
source /opt/ros/humble/setup.zsh
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh
source /home/ujs/ros2_ws/install/setup.zsh
"
```

1. create a package

```bash
cd src
ros2 pkg create my_robot_controller --build-type ament_python --dependencies rclpy
colcon build
```

**NOTE: `pip3 install setuptools==58.2.0`**

minimal python project

```bash
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__("first_node")
        self.get_logger().info("hello world")

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
```

folder structure  is like 

```bash
└── src
    └── my_robot_controller
        ├── my_robot_controller
        │   ├── __init__.py
        │   ├── log
        │   │   ├── COLCON_IGNORE
        │   │   ├── latest -> latest_version-check
        │   │   ├── latest_version-check -> version-check_2023-07-25_10-02-55
        │   │   └── version-check_2023-07-25_10-02-55
        │   │       └── logger_all.log
        │   └── my_first_node.py
        ├── package.xml
        ├── resource
        │   └── my_robot_controller
        ├── setup.cfg
        ├── setup.py # entry point
        └── test
            ├── test_copyright.py
            ├── test_flake8.py
            └── test_pep257.py
```

setpu.py

```bash
entry_points={
          'console_scripts': ['test_node = my_robot_controller.my_first_node:main',
          ]
}
```

and do `colcon build` and `source ~/.zhsrc`

## Timer in ROS

```python
def __init__(self):
          super().__init__("first_node")     
          self.create_timer(1.0,self.timer_callback)

      def timer_callback(self):
          self.get_logger().info("hello")
```

## ROS topic and info

```bash
➜  ros2_ws ros2 topic info /rosout
Type: rcl_interfaces/msg/Log
Publisher count: 2
Subscription count: 0

➜  ros2_ws ros2 interface show rcl_interfaces/msg/Log

➜  ros2_ws ros2 topic echo /rosout
stamp:
  sec: 1690337750
  nanosec: 251515321
level: 20
name: first_node
msg: hello167
file: /home/ujs/ros2_ws/install/my_robot_controller/lib/python3.10/site-packages/my_robot_controller/my_first_node.py
function: timer_callback
line: 11
```

## Simple Publisher mode

```bash
## init a subscriber
ros2_ws ros2 run turtlesim turtlesim_node

➜  ros2_ws ros2 topic list
/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose

➜  ros2_ws ros2 topic info /turtle1/cmd_vel
Type: geometry_msgs/msg/Twist
Publisher count: 0
Subscription count: 1
```

then you have to add dependent packages

`package.xml`

```xml
<depend>rclpy</depend>
<depend>geometry_msgs</depend>
<depend>turtlesim</depend>
```

in code `draw.circle.py`

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DrawCircleNode(Node):
    def __init__(self, node_name="draw_cirlce") -> None:
        super().__init__("draw circle")
        # 10 is queue size. pay attension to "/turtle1/cmd_vel"
        self.cmd_vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.timer = self.create_timer(0.5,self.send_velocity_callback)
        self.get_logger().info("draw circle node has been started")
    def send_velocity_callback(self):
        msg = Twist()
        # (base) ➜  ros2_ws ros2 interface show geometry_msgs/msg/Twist
        # This expresses velocity in free space broken into its linear and angular parts.
        # Vector3  linear float64 x float64 y float64 z
        # Vector3  angular float64 x float64 y float64 z
        msg.linear.x = 2.0
        msg.linear.z = 1.0
        self.cmd_vel_pub.publish(msg)
def main(args=None):
    rclpy.init(args=args)
    node = DrawCircleNode()
    rclpy.spin(node)
    rclpy.shutdown()
```

add node to [`setup.py`](http://setup.py)again

```python
"draw_circle = my_robot_controller.draw_circle:main",
```

```python
**colcon build --symlink-install**
```

## Simple Subscriber node

![Screen Shot 2023-07-27 at 09.51.27.jpg](ROS%20crash%20course%206cfc7e4d15db4169bbd5b45843f73b10/Screen_Shot_2023-07-27_at_09.51.27.jpg)

First get info 

```python
➜  ros2_ws ros2 topic info /turtle1/pose
Type: turtlesim/msg/Pose
Publisher count: 1
Subscription count: 0
```

then write code and add it to `setup.py`

```python
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
```

## Close loop system

receive info from `turtle1/pose` to control through `turtle1/cmd_vel`

![Screen Shot 2023-07-27 at 10.19.48.jpg](ROS%20crash%20course%206cfc7e4d15db4169bbd5b45843f73b10/Screen_Shot_2023-07-27_at_10.19.48.jpg)

```python
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
```

## services

```python
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
```