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
        msg.data = 'Hello, ROS 2 topic!'
        self.pub.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')

def main():
    rclpy.init()
    rclpy.spin(Talker())
    rclpy.shutdown()

if __name__ == '__main__':
    main()