from setuptools import setup

package_name = 'imu_bt_publisher'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Allow `ros2 pkg` to find this package
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # Install package.xml
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Juan Esteban Berón',
    maintainer_email='juan.beron@upb.edu.co',
    description='Bluetooth IMU data publisher for ESP32 → Jetson Nano via RFCOMM.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # This defines the executable name: imu_publisher
            # Run with: ros2 run imu_bt_publisher imu_publisher
            'imu_publisher = imu_bt_publisher.imu_bt_publisher:main',
        ],
    },
)
