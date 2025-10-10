import rclpy
from rclpy.node import Node

def main():
    rclpy.init()
    node = Node("python_node")
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
