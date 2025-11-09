#!/usr/bin/env python3
"""
按键发布节点 - 监听GPIO按键并发布状态到ROS2话题
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device, Button

class ButtonPublisher(Node):
    def __init__(self):
        super().__init__('button_publisher')
        
        # 声明参数
        self.declare_parameter('gpio_pin', 77)
        self.declare_parameter('publish_rate', 10.0)
        
        # 获取参数
        gpio_pin = self.get_parameter('gpio_pin').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # 配置GPIO
        Device.pin_factory = LGPIOFactory(chip=0)
        
        # 创建按键对象
        self.button = Button(gpio_pin)
        self.current_state = False
        
        # 创建发布者
        self.publisher = self.create_publisher(Bool, 'button_state', 10)
        
        # 绑定按键事件
        self.button.when_pressed = self.button_pressed
        self.button.when_released = self.button_released
        
        # 创建定时器定期发布状态
        self.timer = self.create_timer(1.0/publish_rate, self.publish_state)
        
        self.get_logger().info(f'Button publisher started on GPIO {gpio_pin}')
    
    def button_pressed(self):
        """按键按下回调"""
        self.current_state = True
        self.get_logger().info('Button PRESSED')
        self.publish_state()
    
    def button_released(self):
        """按键释放回调"""
        self.current_state = False
        self.get_logger().info('Button RELEASED')
        self.publish_state()
    
    def publish_state(self):
        """发布按键状态"""
        msg = Bool()
        msg.data = self.current_state
        self.publisher.publish(msg)
    
    def destroy_node(self):
        """清理资源"""
        self.button.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ButtonPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()