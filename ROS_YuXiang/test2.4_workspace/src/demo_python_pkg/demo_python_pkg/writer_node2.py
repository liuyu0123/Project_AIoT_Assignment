import rclpy
# from rclpy.node import Node
from demo_python_pkg.person_node2 import PersonNode2

class WriterNode(PersonNode2):
    def __init__(self, node_name: str, name: str, age: int, book: str) -> None:
        super().__init__(node_name, name, age)
        # print('WriterNode is initialized')
        self.book = book

def main():
    rclpy.init()
    node = WriterNode("L_Node","L",18,"Death Note")
    node.eat("apple")
    rclpy.spin(node)
    rclpy.shutdown()