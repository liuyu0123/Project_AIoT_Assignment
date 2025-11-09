from setuptools import find_packages, setup

package_name = 'add_test_srv'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name+"/launch", ['launch/test.launch.py']),
        ('share/' + package_name+"/launch", ['launch/test2.launch.py']),
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
            'talker = add_test_srv.talker:main',
            'listener = add_test_srv.listener:main',
        ],
    },
)
