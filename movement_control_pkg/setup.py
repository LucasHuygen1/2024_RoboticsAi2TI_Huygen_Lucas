from setuptools import setup

package_name = 'movement_control_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/movement_control_pkg_launch_file.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lucas Huygen',
    maintainer_email='...',
    description='Movement control package for robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'movement = movement_control_pkg.movement_control:main', 
        ],
    },
)
