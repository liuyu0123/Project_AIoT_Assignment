import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
import threading
from queue import Queue
import time
import espeakng


class NovelSubNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.novels_queue_ = Queue()
        self.novels_subscriber_ = self.create_subscription(
            String, "novel", self.novel_callback, 10)
        self.speach_thread_ = threading.Thread(target=self.speak_thread)
        self.speach_thread_.start()

    def novel_callback(self, msg):
        self.novels_queue_.put(msg.data)

    def speak_thread(self):
        speaker = espeakng.Speaker()
        speaker.set_voice("zh")
        while rclpy.ok():
            if not self.novels_queue_.empty():
                novel = self.novels_queue_.get()
                self.get_logger().info(f"Speaking {novel}")
                speaker.say(novel)
                speaker.wait()
            else:
                time.sleep(1)
    

def main(args=None):
    rclpy.init(args=args)
    node = NovelSubNode("novel_sub_node")
    rclpy.spin(node)
    rclpy.shutdown()