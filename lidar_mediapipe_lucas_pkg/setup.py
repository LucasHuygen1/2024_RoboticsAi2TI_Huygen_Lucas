from setuptools import setup

package_name = 'lidar_mediapipe_lucas_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/lidar_mediapipe_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A ROS2 package for Mediapipe and Lidar integration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gesture_control = lidar_mediapipe_lucas_pkg.gesture_control:main',
            'lidar_navigation = lidar_mediapipe_lucas_pkg.lidar_navigation:main',
        ],
    },
)

