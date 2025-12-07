from setuptools import setup
import os
from glob import glob

package_name = 'robot_control_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your-email@example.com',
    description='ROS 2 package for humanoid robot control and simulation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_commander_node = robot_control_pkg.joint_commander_node:main',
            'sensor_data_processor_node = robot_control_pkg.sensor_data_processor_node:main',
            'navigation_controller_node = robot_control_pkg.navigation_controller_node:main',
            'manipulation_action_server = robot_control_pkg.manipulation_action_server:main',
        ],
    },
)
