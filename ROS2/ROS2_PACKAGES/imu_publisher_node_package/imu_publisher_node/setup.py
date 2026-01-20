from setuptools import setup

package_name = 'imu_publisher_node'

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
    maintainer='Claudio',
    maintainer_email='claudio@example.com',
    description='Jetson-side IMU publisher node using Bluetooth ESP32.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Executable name: imu_publisher_node
            # Run with: ros2 run imu_publisher_node imu_publisher_node
            'imu_publisher_node = imu_publisher_node.imu_publisher_node:main',
        ],
    },
)
