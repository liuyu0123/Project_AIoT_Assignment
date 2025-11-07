import rclpy
from rclpy.node import Node
from interface_test.srv import NumberAddResult

class Talker(Node):
    def __init__(self):
        super().__init__('add_server')
        self.declare_parameter('weight_a', 1.0)
        self.declare_parameter('weight_b', 1.0)
        self.service = self.create_service(NumberAddResult, 'add_two_number_result', self.add_two_number_result)
        self.get_logger().info('add_server is ready')
    def add_two_number_result(self, request, response):
        weight_a = self.get_parameter('weight_a').value
        weight_b = self.get_parameter('weight_b').value
        response.result = weight_a*request.a + weight_b*request.b
        self.get_logger().info(f'收到请求: {request.a} + {request.b} = {response.result}')
        return response
    
def main(args=None):
    rclpy.init(args=args)
    talker = Talker()
    rclpy.spin(talker)
    rclpy.shutdown()