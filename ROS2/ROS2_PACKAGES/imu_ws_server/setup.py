from setuptools import setup

package_name = 'imu_ws_server'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Allows `ros2 pkg` to find the package
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # Install package.xml
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Claudio',
    maintainer_email='claudio@example.com',
    description='ROS 2 node that exposes IMU data over a WebSocket server.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # CLI entry point: `ros2 run imu_ws_server imu_ws_server_node`
            'imu_ws_server_node = imu_ws_server.imu_ws_server_node:main',
        ],
    },
)
