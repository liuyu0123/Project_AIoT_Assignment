import rclpy
from rclpy.node import Node
from interface_test.msg import NumberAdd

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.sub = self.create_subscription(NumberAdd, 'add_two_number', self.callback, 10)
        self.get_logger().info('listener start')

    def callback(self, msg):
        sum = msg.a + msg.b
        self.get_logger().info('%d + %d = %d' % (msg.a, msg.b, sum))


def main(args=None):
    rclpy.init(args=args)
    node = Listener()
    rclpy.spin(node)
    rclpy.shutdown()