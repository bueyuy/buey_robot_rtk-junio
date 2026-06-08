"""MqttWaypointsOutput: publica waypoints por MQTT (retained).

Clase reutilizable (no nodo). WaypointManager/TrajectoryController la llama al cargar waypoints.
Topics publicados (retained):
  - bueyuy/waypoints_xy  (JSON {waypoints: [{x, y}, ...]})
  - bueyuy/waypoints     (JSON {waypoints_gps: [{lat, lon}, ...]}) -- si se proveen GPS
"""

import json

from buey_robot.adapters.mqtt.client import MqttClient
from buey_robot.utils.config import require_key


class MqttWaypointsOutput:
    """Publica waypoints por MQTT con retain=True."""

    def __init__(self, client: MqttClient, mqtt_cfg: dict):
        """
        Args:
            client:   Instancia compartida de MqttClient (de get_client()).
            mqtt_cfg: Diccionario de mqtt.yaml.
        """
        self._client = client
        self.topic_waypoints = require_key(mqtt_cfg, 'topics', 'waypoints')
        self.topic_waypoints_xy = require_key(mqtt_cfg, 'topics', 'waypoints_xy')

    def send_xy(self, waypoints: list):
        """Publica waypoints en coordenadas locales (retained).

        Args:
            waypoints: Lista de tuplas (x, y) o dicts {'x': x, 'y': y}.
        """
        if waypoints and isinstance(waypoints[0], tuple):
            wps = [{'x': wx, 'y': wy} for wx, wy in waypoints]
        else:
            wps = list(waypoints)
        payload = json.dumps({'waypoints': wps})
        self._client.publish(self.topic_waypoints_xy, payload, retain=True)

    def send_gps(self, waypoints_gps: list):
        """Publica waypoints en coordenadas GPS (retained).

        Args:
            waypoints_gps: Lista de dicts {'lat': ..., 'lon': ...}.
        """
        payload = json.dumps({'waypoints_gps': waypoints_gps})
        self._client.publish(self.topic_waypoints, payload, retain=True)
