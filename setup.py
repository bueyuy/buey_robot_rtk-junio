from setuptools import setup, find_namespace_packages
import os
from glob import glob

package_name = 'buey_robot'

# config/ espeja la estructura de buey_robot/; instalar preservando subcarpetas.
config_data = [
    (os.path.join('share', package_name, root),
     [os.path.join(root, f) for f in files if f.endswith('.yaml')])
    for root, _, files in os.walk('config')
    if any(f.endswith('.yaml') for f in files)
]

setup(
    name=package_name,
    version='2.0.0',
    # Sin __init__.py: paquetes namespace (PEP 420).
    packages=find_namespace_packages(include=['buey_robot', 'buey_robot.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ] + config_data,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@todo.todo',
    description='Robot Buey V RTK - navegacion autonoma outdoor (GPS RTK), arquitectura por capas',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Navigation
            'navigation_controller = buey_robot.navigation.controller:main',
            'navigation_initializer = buey_robot.navigation.initializer:main',
            'joystick_controller = buey_robot.navigation.joystick:main',

            # Odometry
            'odometry_gps = buey_robot.odometry.gps:main',

            # Drivers (especificos del modelo) -> contrato /imu/*, /gps/fix
            'imu_mpu6050 = buey_robot.drivers.imu_mpu6050:main',
            'gps_nmea = buey_robot.drivers.gps_nmea:main',

            # Fusion (agnostica del sensor): /imu/yaw + /gps/course -> /heading/fused
            'fusion_heading = buey_robot.fusion.heading:main',

            # Motor
            'motor_gateway = buey_robot.motor.gateway:main',

            # Adapters MQTT: bridges (nodos transport ROS <-> MQTT)
            'command_bridge = buey_robot.adapters.mqtt.command_bridge:main',
            'telemetry_bridge = buey_robot.adapters.mqtt.telemetry_bridge:main',
        ],
    },
)
