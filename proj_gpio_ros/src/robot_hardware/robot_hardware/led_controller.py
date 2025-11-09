#!/usr/bin/env python3
"""
LED控制节点 - 订阅ROS2话题控制GPIO LED
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device, LED

class LEDController(Node):
    def __init__(self):
        super().__init__('led_controller')
        
        # 声明参数
        self.declare_parameter('gpio_pin', 71)
        self.declare_parameter('initial_state', False)
        
        # 获取参数
        gpio_pin = self.get_parameter('gpio_pin').value
        initial_state = self.get_parameter('initial_state').value
        
        # 配置GPIO
        Device.pin_factory = LGPIOFactory(chip=0)
        
        # 创建LED对象
        self.led = LED(gpio_pin)
        
        # 设置初始状态
        if initial_state:
            self.led.on()
        else:
            self.led.off()
        
        # 创建订阅者
        self.subscription = self.create_subscription(
            Bool,
            'led_control',
            self.control_callback,
            10
        )
        
        self.get_logger().info(f'LED controller started on GPIO {gpio_pin}')
    
    def control_callback(self, msg):
        """LED控制回调"""
        if msg.data:
            self.led.on()
            self.get_logger().info('LED turned ON')
        else:
            self.led.off()
            self.get_logger().info('LED turned OFF')
    
    def destroy_node(self):
        """清理资源"""
        self.led.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LEDController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()