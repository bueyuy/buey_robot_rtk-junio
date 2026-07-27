"""CommandBridge: relay puro MQTT -> ROS. La web publica el formato ROS a bueyuy/<topic>
y esto lo reenvia verbatim al topic ROS. NO convierte coordenadas (eso lo hace OdometryGps)."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Empty

from buey_robot.adapters.mqtt.client import get_client
from buey_robot.utils.config import load_config
from buey_robot.contracts import GEO_ROUTE, NAV_START


class CommandBridge(Node):
    def __init__(self):
        super().__init__('command_bridge')
        mqtt_cfg = load_config('mqtt.yaml')
        self._mqtt = get_client(mqtt_cfg, logger=self.get_logger())

        # Salidas: topics ROS
        # /geo/route retiene el ultimo valor: si OdometryGps arranca despues, igual lo recibe.
        retained = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._route_pub = self.create_publisher(String, GEO_ROUTE, retained)
        self._start_pub = self.create_publisher(Empty, NAV_START, 10)

        # Entradas: broker MQTT
        self._mqtt.subscribe('bueyuy' + GEO_ROUTE, self._on_route)
        self._mqtt.subscribe('bueyuy' + NAV_START, self._on_start)
        self.get_logger().info('command_bridge iniciado (bueyuy/geo/route, bueyuy/nav/start -> ROS)')

    def _on_route(self, client, userdata, msg):
        self._route_pub.publish(String(data=msg.payload.decode()))

    def _on_start(self, client, userdata, msg):
        self._start_pub.publish(Empty())


def main(args=None):
    rclpy.init(args=args)
    node = CommandBridge()
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
