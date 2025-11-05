import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.pub = self.create_publisher(String, 'hello', 10)
        self.timer = self.create_timer(1.0, self.timer_cb)

    def timer_cb(self):
        msg = String()
        msg.data = 'Hello World'
        self.pub.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main():
    rclpy.init()
    node = Talker()
    rclpy.spin(node)
    rclpy.shutdown()