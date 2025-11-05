import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.sub = self.create_subscription(String, 'hello', self.callback, 10)

    def callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main():
    rclpy.init()
    rclpy.spin(Listener())
    rclpy.shutdown()

if __name__ == '__main__':
    main()