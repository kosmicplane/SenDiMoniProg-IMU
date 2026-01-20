from setuptools import setup

package_name = 'imu_ws_client'

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
    description='ROS 2 node that subscribes to IMU data from a WebSocket server and republishes it.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # CLI entry point: `ros2 run imu_ws_client imu_ws_client_node`
            'imu_ws_client_node = imu_ws_client.imu_ws_client_node:main',
        ],
    },
)
