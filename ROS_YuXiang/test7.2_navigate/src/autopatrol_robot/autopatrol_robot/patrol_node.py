import rclpy
from geometry_msgs.msg import PoseStamped, Pose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros import TransformListener, Buffer
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from rclpy.duration import Duration
from autopatrol_interfaces.srv import SpeachText


class PatrolNode(BasicNavigator):
    def __init__(self, node_name="patrol_node"):
        super().__init__(node_name)
        # 导航相关定义
        self.declare_parameter('initial_point', [0.0, 0.0, 0.0])
        self.declare_parameter('target_points', [0.0, 0.0, 0.0, 1.0, 1.0, 1.57])
        self.initial_point_ = self.get_parameter('initial_point').value
        self.target_points_ = self.get_parameter('target_points').value
        self.buffer_ = Buffer()
        self.listener_ = TransformListener(self.buffer_, self)
        self.speach_client_ = self.create_client(SpeachText, 'speach_text')


    def get_pose_by_xyyaw(self, x, y, yaw):
        # 通过x,y,yaw 合成 PoseStamped
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w, pose.pose.orientation.x,\
            pose.pose.orientation.y, pose.pose.orientation.z = quaternion_from_euler(0, 0, yaw)
        return pose


    def init_robot_pose(self):
        # 初始化机器人位置
        # 从参数获取初始位置
        self.initial_point_ = self.get_parameter('initial_point').value
        # 合成位姿并设置
        self.setInitialPose(self.get_pose_by_xyyaw(\
            self.initial_point_[0], self.initial_point_[1], self.initial_point_[2]))
        # 等待导航激活
        self.waitUntilNav2Active()


    def get_target_points(self):
        # 通过参数获取目标点集合
        points = []
        self.target_points_ = self.get_parameter('target_points').value
        for index in range(int(len(self.target_points_) / 3)):
            x = self.target_points_[index * 3]
            y = self.target_points_[index * 3 + 1]
            yaw = self.target_points_[index * 3 + 2]
            points.append([x, y, yaw])
            self.get_logger().info(f'target point {index} --> ({x}, {y}, {yaw})')
        return points


    def nav_to_pose(self, target_pose):
        # 导航到指定位置
        self.waitUntilNav2Active()
        result = self.goToPose(target_pose)
        while not self.isTaskComplete():
            feedback = self.getFeedback()
            if feedback:
                self.get_logger().info(
                    f'will arrive after {Duration.from_msg(feedback.estimate_time_remaining).nanoseconds / 1e9} seconds')
        # 最终结果判断
        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('Goal was canceled')
        elif result == TaskResult.FAILED:
            self.get_logger().error('Goal failed')
        else:
            self.get_logger().error('the result is invalid')


    def get_current_pose(self):
        # 通过TF获取当前位置
        while rclpy.ok():
            try:
                tf = self.buffer.lookup_transform(
                    'map', 'base_footprint', rclpy.time.Time(seconds=0),
                    rclpy.time.Duration(seconds=1))
                transform = tf.transform
                rotation_euler = euler_from_quaternion([
                    transform.rotation.x,
                    transform.rotation.y,
                    transform.rotation.z,
                    transform.rotation.w])
                self.get_logger().info(
                    f'translate: ({transform.translation}, ',
                    f'rotation: {transform.rotation}, ', 
                    f'euler: {rotation_euler}.')
                return transform
            except Exception as e:
                self.get_logger().warn(f'get current pose failed: {str(e)}')
                rclpy.sleep(0.1)


    def speach_text(self, text):
        # 发送文字到语音合成
        while not self.speach_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service not available, waiting again...')
        request = SpeachText.Request()
        request.text = text
        future = self.speach_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            result = future.result().result
            if result:
                self.get_logger().info(f'speach succeed: {text}')
            else:
                self.get_logger().warn(f'speach failure: {text}')
        else:
            self.get_logger().warn('exception while calling service')



def main(args=None):
    rclpy.init(args=args)
    patrol_node = PatrolNode()
    patrol_node.speach_text(text = '正在初始化位置')
    patrol_node.init_robot_pose()
    patrol_node.speach_text(text = '位置初始化完成')

    while rclpy.ok():
        # target_points = patrol_node.get_target_points()
        # for target_point in target_points:
        for target_point in patrol_node.get_target_points():
            x, y, yaw = target_point[0], target_point[1], target_point[2]
            target_pose = patrol_node.get_pose_by_xyyaw(x, y, yaw)
            patrol_node.speach_text(text = f'正在导航到目标点 ({x}, {y}, {yaw})')
            patrol_node.nav_to_pose(target_pose)
    rclpy.shutdown()