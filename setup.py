from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'buey_robot'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['deprecated', 'tools']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'waypoints'), glob('waypoints/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@todo.todo',
    description='Robot Buey V RTK - navegacion autonoma con ZED/RTK, arquitectura por capas',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Navigation
            'trajectory_controller = buey_robot.navigation.controller:main',
            'joystick_controller = buey_robot.navigation.joystick:main',

            # Odometry
            'zed_odometry = buey_robot.odometry.zed:main',
            'rtk_odometry = buey_robot.odometry.rtk:main',

            # Sensores (drivers de hardware)
            # IMU LSM303: micro-ROS publica el campo crudo /imu/mag; imu_compass -> heading.
            # IMU MPU6050: micro-ROS publica /imu/data (accel+gyro); mpu6050_gyro -> bias + heading yaw.
            'gps_nmea_driver = buey_robot.mapper.gps_nmea:main',
            'imu_compass = buey_robot.mapper.imu_compass:main',
            'mpu6050_gyro = buey_robot.mapper.mpu6050_gyro:main',

            # Motor
            'motor_gateway = buey_robot.motor.gateway:main',

            # Adapters MQTT outputs (nodos)
            'pose_publisher = buey_robot.adapters.mqtt.outputs.pose:main',
            'imu_bridge = buey_robot.adapters.mqtt.outputs.imu_bridge:main',
        ],
    },
)
