import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from tf_transformations import quaternion_from_euler, euler_from_quaternion

class TFListener(Node):
    def __init__(self):
        super().__init__('tf2_listener')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.timer = self.create_timer(1.0, self.get_transform)

    def get_transform(self):
        try:
            tf = self.buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time(seconds=0),
                rclpy.duration.Duration(seconds=1))
            transform = tf.transform
            rotate = euler_from_quaternion([transform.rotation.x,
                                            transform.rotation.y,
                                            transform.rotation.z,
                                            transform.rotation.w])
            self.get_logger().info('x: {:.2f}, y: {:.2f}, z: {:.2f}, roll: {:.2f}, pitch: {:.2f}, yaw: {:.2f}'.format(
                transform.translation.x,
                transform.translation.y,
                transform.translation.z,
                rotate[0],
                rotate[1],
                rotate[2]))
        except Exception as e:
            self.get_logger().warn(f'can not receive transform, because: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = TFListener()
    rclpy.spin(node)
    rclpy.shutdown()