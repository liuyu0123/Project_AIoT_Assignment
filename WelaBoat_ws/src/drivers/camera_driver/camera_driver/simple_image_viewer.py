# simple_image_viewer/simple_image_viewer/viewer.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys

class SimpleImageViewer(Node):
    def __init__(self, topic_name='/camera/left/image_raw'):
        super().__init__('simple_image_viewer')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, topic_name, self.listener_callback, 10)
        self.get_logger().info(f'Subscribed to {topic_name}')

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow("Simple Image Viewer", cv_image)
            if cv2.waitKey(1) == ord('q'):  # 按 q 退出
                rclpy.shutdown()
        except Exception as e:
            self.get_logger().error(f'CV error: {e}')

def main(args=None):
    rclpy.init(args=args)
    topic = sys.argv[1] if len(sys.argv) > 1 else '/camera/left/image_raw'
    viewer = SimpleImageViewer(topic)
    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    viewer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()