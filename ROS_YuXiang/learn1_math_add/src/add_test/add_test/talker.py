import rclpy
from rclpy.node import Node
from interface_test.msg import NumberAdd

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(NumberAdd, 'add_two_number', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = NumberAdd()
        msg.a = float(self.i)
        msg.b = float(self.i + 1.0)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)
        self.i += 2

def main(args=None):
    rclpy.init(args=args)
    talker = Talker()
    rclpy.spin(talker)
    rclpy.shutdown()
