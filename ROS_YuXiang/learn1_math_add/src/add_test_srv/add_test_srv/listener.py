import rclpy
from rclpy.node import Node
from interface_test.srv import NumberAddResult
import sys


def extract_numeric_args(argv):
    """
    从sys.argv中提取launch通过arguments传递的参数
    跳过ROS自动添加的--ros-args部分
    """
    numeric_args = []
    for arg in argv[1:]:  # 跳过程序名
        if arg.startswith('--'):  # ROS参数开始，停止收集
            break
        numeric_args.append(arg)
    return numeric_args


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
                f'listener.send_request 结果: {request.a} + {request.b} = {response.result}'
            )
            return response.result
        else:
            self.get_logger().error('服务调用失败')
            return None
        
def main(args=None):
    # 在rclpy.init之前提取参数
    if args is None:
        args = sys.argv
    # 提取数值参数
    launch_args = extract_numeric_args(args)
    a_default = 1.0
    b_default = 2.0
    if len(launch_args) < 2:
        a, b = a_default, b_default
        # Node.get_logger.info(f'[INFO] 参数不足，使用默认值: a={a}, b={b}')
    else:
        a, b = float(launch_args[0]), float(launch_args[1])
        # Node.get_logger.info(f'[INFO] 使用参数: a={a}, b={b}')

    rclpy.init(args=args)
    listener = Listener()
    if False:
        # if argv_len := len(sys.argv) != 3:
        argv_len = len(sys.argv)
        if argv_len != 3:
            listener.get_logger().info(f'使用默认参数: {a_default} + {b_default}')
            result = listener.send_request(a_default, b_default)  # 调用服务，注意这里不是用的spin
        else:
            result = listener.send_request(sys.argv[1], sys.argv[2])
    listener.get_logger().info(f'使用参数: {a} + {b}')
    result = listener.send_request(a, b)
    listener.get_logger().info(f'结果(in main fun): {result}')
    rclpy.shutdown()