import rclpy
from rclpy.node import Node
import requests
from example_interfaces.msg import String
from queue import Queue

class NovelPubNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.novels_queue_ = Queue()

        self.novels_publisher_ = self.create_publisher(String, "novels", 10)
        self.timer_ = self.create_timer(5.0, self.timer_callback)

    def download_novel(self, url):
        response = requests.get(url)
        response.encoding = "utf-8"
        self.get_logger().info(f"Download novel finished from {url}")
        for line in response.text.splitlines():
            self.novels_queue_.put(line)

    def timer_callback(self):
        if not self.novels_queue_.empty():
            msg = String()
            msg.data = self.novels_queue_.get()
            self.novels_publisher_.publish(msg)
            self.get_logger().info(f"Published novel: {msg.data}")

def main():
    rclpy.init()
    node = NovelPubNode("novel_pub_node")
    node.download_novel("http://localhost:8000/novel1.txt")
    rclpy.spin(node)
    rclpy.shutdown()