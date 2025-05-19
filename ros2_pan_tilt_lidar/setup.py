from setuptools import setup

package_name = 'ros2_pan_tilt_lidar'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='A ROS2 package for controlling a pan-tilt gimbal and lidar measurement.',
    license='License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pan_tilt_lidar_node = pan_tilt_lidar.node_main:main',
        ],
    },
)