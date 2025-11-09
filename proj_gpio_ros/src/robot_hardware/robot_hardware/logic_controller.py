#!/usr/bin/env python3
"""
逻辑控制节点 - 实现按键到硬件的控制逻辑
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

class LogicController(Node):
    def __init__(self):
        super().__init__('logic_controller')
        
        # 创建订阅者（订阅按键状态）
        self.button_sub = self.create_subscription(
            Bool,
            'button_state',
            self.button_callback,
            10
        )
        
        # 创建发布者（发布LED和舵机控制命令）
        self.led_pub = self.create_publisher(Bool, 'led_control', 10)
        self.servo_pub = self.create_publisher(Float32, 'servo_control', 10)
        
        self.get_logger().info('Logic controller started')
    
    def button_callback(self, msg):
        """按键状态回调 - 实现控制逻辑"""
        if msg.data:  # 按键按下
            self.get_logger().info('Button pressed - Activating outputs')
            
            # 点亮LED
            led_msg = Bool()
            led_msg.data = True
            self.led_pub.publish(led_msg)
            
            # 舵机转到0.5位置
            servo_msg = Float32()
            servo_msg.data = 0.5
            self.servo_pub.publish(servo_msg)
            
        else:  # 按键释放
            self.get_logger().info('Button released - Deactivating outputs')
            
            # 熄灭LED
            led_msg = Bool()
            led_msg.data = False
            self.led_pub.publish(led_msg)
            
            # 舵机归位到0位置
            servo_msg = Float32()
            servo_msg.data = 0.0
            self.servo_pub.publish(servo_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LogicController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()