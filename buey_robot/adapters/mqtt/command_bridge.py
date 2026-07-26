"""CommandBridge: trae comandos de afuera (MQTT) al stack ROS. Parsea la ruta de
waypoints (lat/lon) y el GO, y los publica como topics ROS. Transporte puro: NO
convierte coordenadas (eso lo hace OdometryGps, el unico bilingue)."""

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Empty

from buey_robot.adapters.mqtt.client import get_client
from buey_robot.utils.config import load_config, require_key
from buey_robot.contracts import GEO_ROUTE, NAV_START


class CommandBridge(Node):
    def __init__(self):
        super().__init__('command_bridge')
        mqtt_cfg = load_config('mqtt.yaml')
        self._mqtt = get_client(mqtt_cfg, logger=self.get_logger())

        # /geo/route retiene el ultimo valor: si OdometryGps arranca despues, igual lo recibe.
        retained = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._route_pub = self.create_publisher(String, GEO_ROUTE, retained)
        self._start_pub = self.create_publisher(Empty, NAV_START, 10)

        self._mqtt.subscribe(require_key(mqtt_cfg, 'topics', 'waypoints'), self._on_waypoints)
        self._mqtt.subscribe(require_key(mqtt_cfg, 'topics', 'nav_start'), self._on_start)

    def _on_waypoints(self, client, userdata, msg):
        route = self._parse(msg)
        if route is None:                  # payload invalido -> no republicar (lista vacia SI: idle)
            return
        self._route_pub.publish(String(data=json.dumps(route)))

    def _on_start(self, client, userdata, msg):
        self._start_pub.publish(Empty())

    @staticmethod
    def _parse(msg):
        # MQTT: {waypoints_gps:[{lat,lon}], loop} -> ROS: {waypoints:[{lat,lon}], loop}
        try:
            raw = json.loads(msg.payload.decode())
            points = raw.get('waypoints_gps')
            if not isinstance(points, list):
                return None
            wps = [{'lat': float(p['lat']), 'lon': float(p['lon'])} for p in points
                   if isinstance(p, dict) and 'lat' in p and 'lon' in p]
            return {'waypoints': wps, 'loop': bool(raw.get('loop', False))}
        except Exception:
            return None


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
