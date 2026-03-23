from setuptools import find_packages, setup

package_name = 'offboard_control_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/offboard_control.launch.py']),
        ('share/' + package_name + '/config', ['config/offboard_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PX4 Maintainer',
    maintainer_email='maintainer@example.com',
    description='PX4 offboard control module using ROS2 architecture.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offboard_control_node = offboard_control_ros2.offboard_node:main',
        ],
    },
)
