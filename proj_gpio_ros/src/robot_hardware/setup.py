from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_hardware'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 添加 launch 文件
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='riba',
    maintainer_email='liuyu0123@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'button_publisher = robot_hardware.button_publisher:main',
            'led_controller = robot_hardware.led_controller:main',
            'servo_controller = robot_hardware.servo_controller:main',
            'logic_controller = robot_hardware.logic_controller:main',
        ],
    },
)
