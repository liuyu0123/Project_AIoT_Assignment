import rclpy
from rclpy.node import Node
from chapt4_interfaces.srv import FaceDetector
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge

class FaceDetectorClient(Node):
    def __init__(self):
        super().__init__('face_detect_client')
        self.client = self.create_client(FaceDetector, '/face_detect')
        self.bridge = CvBridge()
        self.test1_image_path = get_package_share_directory(\
            'demo_python_service') + '/resource/test1.jpg'
        self.image = cv2.imread(self.test1_image_path)

    def send_request(self):
        # 1. 判断服务是否可用
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        # 2. 创建请求对象
        request = FaceDetector.Request()
        request.image = self.bridge.cv2_to_imgmsg(self.image)
        # 3. 发送请求
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        # 4. 处理响应
        response = future.result()
        self.get_logger().info(
            f"Response: {response.number} faces detected, cost {response.use_time} ms")
        self.show_face_locations(response)

    def show_face_locations(self, response):
        for i in range(response.number):
            top = response.top[i]
            right = response.right[i]
            bottom = response.bottom[i]
            left = response.left[i]
            cv2.rectangle(self.image, (left, top), (right, bottom), (225, 0, 0), 2)
        cv2.imshow('Face Detection Result', self.image)
        cv2.waitKey(0)

def main(args=None):
    rclpy.init(args=args)
    client = FaceDetectorClient()
    client.send_request()
    rclpy.shutdown()