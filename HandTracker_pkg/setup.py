from setuptools import setup

package_name = 'HandTracker_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gesture_control_launch.py']),  
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lucas Huygen',
    maintainer_email='...',
    description='moving turtlebot with hand tracker.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'HandTracker = HandTracker_pkg.HandTracker:main',
        ],
    },
)

