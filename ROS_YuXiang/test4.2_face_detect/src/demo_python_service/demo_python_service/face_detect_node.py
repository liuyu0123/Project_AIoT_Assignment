import rclpy
from rclpy.node import Node
from chapt4_interfaces.srv import FaceDetector
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import cv2
import face_recognition
import time
from rcl_interfaces.msg import SetParametersResult

class FaceDetectorionNode(Node):
    def __init__(self):
        super().__init__("face_detection_node")
        self.bridge = CvBridge()
        self.service = self.create_service(FaceDetector, "/face_detect", self.detect_face_callback)
        self.default_image_path = get_package_share_directory('demo_python_service') \
            + '/resource/default.jpg'
        self.get_logger().info("face detector has been started")
        # self.upsample_times = 1
        # self.model = "hog"

        self.declare_parameter("face_locations_upsample_times", 1)
        self.declare_parameter("face_locations_model", "hog")
        self.model = self.get_parameter("face_locations_model").value
        self.upsample_times = self.get_parameter("face_locations_upsample_times").value
        self.add_on_set_parameters_callback(self.parameters_callback)

    def detect_face_callback(self, request, response):
        if request.image.data:
            cv_image = self.bridge.imgmsg_to_cv2(request.image)
        else:
            cv_image = cv2.imread(self.default_image_path)
        start_time = time.time()
        self.get_logger().info("start detecting face")
        face_locations = face_recognition.face_locations(\
            cv_image, number_of_times_to_upsample=self.upsample_times, model=self.model)
        end_time = time.time()
        response.number = len(face_locations)
        response.use_time = end_time - start_time
        for top, right, bottom, left in face_locations:
            response.top.append(top)
            response.right.append(right)
            response.bottom.append(bottom)
            response.left.append(left)
        
        return response
    
    def parameters_callback(self, params):
        for param in params:
            self.get_logger().info("{} has been set to {}".format(param.name, param.value))
            if param.name == "face_locations_upsample_times":
                self.upsample_times = param.value
            elif param.name == "face_locations_model":
                self.model = param.value
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectorionNode()
    rclpy.spin(node)
    rclpy.shutdown()