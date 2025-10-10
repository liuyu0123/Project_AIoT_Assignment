import rclpy
from rclpy.node import Node

class PersonNode2(Node):
    def __init__(self, node_name: str, name: str, age: int) -> None:
        super().__init__(node_name)
        # print(f"{name} is {age} years old")
        self.name = name
        self.age = age

    def eat(self, food: str) -> None:
        self.get_logger().info(f"{self.name} is eating {food}")
        # print(f"{self.name} is eating {food}")

def main():
    rclpy.init()
    node = PersonNode2('person_node', "Alice", 18)
    node.eat("apple")
    rclpy.spin(node)
    rclpy.shutdown()
        