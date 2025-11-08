import rclpy
from rclpy.node import Node
from interface_test.srv import NumberAddResult
import sys

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.client = self.create_client(NumberAddResult, 'add_two_number_result')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        

    def send_request(self, a, b):
        """发送请求并等待结果（同步方式）"""
        request = NumberAddResult.Request()
        request.a = float(a)
        request.b = float(b)
        
        # 异步发送请求，返回 Future 对象
        future = self.client.call_async(request)
        
        # 阻塞等待结果（同步）
        rclpy.spin_until_future_complete(self, future)
        
        # 处理响应
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(
                f'结果: {request.a} + {request.b} = {response.result}'
            )
            return response.result
        else:
            self.get_logger().error('服务调用失败')
            return None
        
def main(args=None):
    rclpy.init(args=args)
    listener = Listener()
    a_default = 1.0
    b_default = 2.0
    if argv_len := len(sys.argv) != 3:
        listener.get_logger().info(f'使用默认参数: {a_default} + {b_default}')
        result = listener.send_request(a_default, b_default)  # 调用服务，注意这里不是用的spin
    else:
        result = listener.send_request(sys.argv[1], sys.argv[2])
    listener.get_logger().info(f'结果(in main fun): {result}')
    rclpy.shutdown()