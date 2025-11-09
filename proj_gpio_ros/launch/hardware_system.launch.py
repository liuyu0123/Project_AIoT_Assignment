#!/usr/bin/env python3
"""
硬件系统启动文件 - 一次性启动所有节点
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 按键发布节点
        Node(
            package='robot_hardware',
            executable='button_publisher',
            name='button_publisher',
            parameters=[{
                'gpio_pin': 77,
                'publish_rate': 10.0
            }],
            output='screen'
        ),
        
        # LED控制节点
        Node(
            package='robot_hardware',
            executable='led_controller',
            name='led_controller',
            parameters=[{
                'gpio_pin': 71,
                'initial_state': False
            }],
            output='screen'
        ),
        
        # 舵机控制节点
        Node(
            package='robot_hardware',
            executable='servo_controller',
            name='servo_controller',
            parameters=[{
                'gpio_pin': 70,
                'min_pulse_width': 0.0005,
                'max_pulse_width': 0.0025,
                'frame_width': 0.02,
                'detach_delay': 0.3
            }],
            output='screen'
        ),
        
        # 逻辑控制节点
        Node(
            package='robot_hardware',
            executable='logic_controller',
            name='logic_controller',
            output='screen'
        ),
    ])