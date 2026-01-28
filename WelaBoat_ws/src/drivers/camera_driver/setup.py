from setuptools import find_packages, setup
import os

package_name = 'camera_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'utils'), 
         [os.path.join('camera_driver', 'utils', 'stereoParams_512pixel.json')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='riba',
    maintainer_email='liuyu0123@outlook.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_driver = camera_driver.camera_driver_node:main',  # 节点入口
            'camera_viewer = camera_driver.simple_image_viewer:main',  # 预览相机图像
            'camera_rectified = camera_driver.camera_rectify_node:main', # 相机校正后图像
        ],
    },
)
