"""Odometria ZED: filtra y re-publica odometria visual en /odom_filtered.

Abstrae la fuente de odometria para que navigation/controller.py
no sepa si viene de ZED o de RTK.
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import math

from buey_robot.utils.math import quaternion_to_yaw
from buey_robot.utils.filters import MovingAverageFilter


class ZedOdometry(Node):
    def __init__(self):
        super().__init__('zed_odometry')

        # --- Parametros de odometria ZED (ROS2 estandar, sin defaults) ---
        self.declare_parameter('odometry.topic', Parameter.Type.STRING)
        self.declare_parameter('camera.offset_x_m', Parameter.Type.DOUBLE)
        self.declare_parameter('filter.enabled', Parameter.Type.BOOL)
        self.declare_parameter('filter.window_size', Parameter.Type.INTEGER)

        odom_topic = self.get_parameter('odometry.topic').value
        self.camera_offset_x = self.get_parameter('camera.offset_x_m').value
        self.filter_enabled = self.get_parameter('filter.enabled').value
        window = self.get_parameter('filter.window_size').value
        self.x_filter = MovingAverageFilter(window) if self.filter_enabled else None
        self.y_filter = MovingAverageFilter(window) if self.filter_enabled else None

        self.current_x = None
        self.current_y = None
        self.current_heading = None
        self.odom_received = False

        # Salida unificada: /odom_filtered (lo consume navigation/controller.py y adapters/mqtt/outputs/pose.py)
        self.odom_filtered_pub = self.create_publisher(Odometry, '/odom_filtered', 10)
        self.heading_pub = self.create_publisher(Float64, '/heading/zed', 10)
        self.heading_imu_pub = self.create_publisher(Float64, '/heading/imu', 10)

        self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.create_subscription(Imu, '/zed/zed_node/imu/data', self.imu_callback, 10)

        self.get_logger().info('ZED Odometry iniciado')
        self.get_logger().info(f'  Odom: {odom_topic}, camera offset: {self.camera_offset_x:.3f}m')

    def odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self.current_heading = quaternion_to_yaw(q.x, q.y, q.z, q.w)

        cam_x = msg.pose.pose.position.x
        cam_y = msg.pose.pose.position.y
        x = cam_x - self.camera_offset_x * math.cos(self.current_heading)
        y = cam_y - self.camera_offset_x * math.sin(self.current_heading)

        if self.filter_enabled and self.x_filter and self.y_filter:
            x = self.x_filter.update(x)
            y = self.y_filter.update(y)

        self.current_x = x
        self.current_y = y

        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info(
                f'Primer odom: cam=({cam_x:.2f}, {cam_y:.2f}), '
                f'centro=({x:.2f}, {y:.2f}), '
                f'heading={math.degrees(self.current_heading):.1f} deg'
            )

        heading_msg = Float64()
        heading_msg.data = math.degrees(self.current_heading)
        self.heading_pub.publish(heading_msg)

        # Publicar odometria filtrada a /odom_filtered
        filtered = Odometry()
        filtered.header = msg.header
        filtered.child_frame_id = msg.child_frame_id
        filtered.pose.pose.position.x = self.current_x
        filtered.pose.pose.position.y = self.current_y
        filtered.pose.pose.position.z = msg.pose.pose.position.z
        filtered.pose.pose.orientation = msg.pose.pose.orientation
        filtered.twist = msg.twist
        self.odom_filtered_pub.publish(filtered)

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        heading_msg = Float64()
        heading_msg.data = math.degrees(yaw)
        self.heading_imu_pub.publish(heading_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ZedOdometry()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
