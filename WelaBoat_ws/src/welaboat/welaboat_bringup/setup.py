from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'welaboat_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('welaboat_bringup', 'launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('welaboat_bringup', 'config', '*yaml'))),
        (os.path.join('share', package_name, 'map'),
         glob(os.path.join('welaboat_bringup', 'map', '*'))),
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
        ],
    },
)
