#!/usr/bin/env python3
"""
舵机控制节点 - 订阅ROS2话题控制PWM舵机
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time

from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device, Servo

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        
        # 声明参数
        self.declare_parameter('gpio_pin', 70)
        self.declare_parameter('min_pulse_width', 0.0005)
        self.declare_parameter('max_pulse_width', 0.0025)
        self.declare_parameter('frame_width', 0.02)
        self.declare_parameter('detach_delay', 0.3)
        
        # 获取参数
        gpio_pin = self.get_parameter('gpio_pin').value
        min_pw = self.get_parameter('min_pulse_width').value
        max_pw = self.get_parameter('max_pulse_width').value
        frame_w = self.get_parameter('frame_width').value
        self.detach_delay = self.get_parameter('detach_delay').value
        
        # 配置GPIO
        Device.pin_factory = LGPIOFactory(chip=0)
        
        # 创建舵机对象
        self.servo = Servo(
            gpio_pin,
            min_pulse_width=min_pw,
            max_pulse_width=max_pw,
            frame_width=frame_w
        )
        
        # 创建订阅者
        self.subscription = self.create_subscription(
            Float32,
            'servo_control',
            self.control_callback,
            10
        )
        
        self.get_logger().info(f'Servo controller started on GPIO {gpio_pin}')
    
    def control_callback(self, msg):
        """舵机控制回调"""
        value = msg.data
        
        # 值范围检查
        if -1.0 <= value <= 1.0:
            self.servo.value = value
            self.get_logger().info(f'Servo set to {value:.2f}')
            
            # 延迟后断开信号防止抖动
            time.sleep(self.detach_delay)
            self.servo.detach()
        else:
            self.get_logger().warning(
                f'Invalid servo value {value}. Must be between -1.0 and 1.0'
            )
    
    def destroy_node(self):
        """清理资源"""
        self.servo.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ServoController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()