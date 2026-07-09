"""MqttOriginOutput: publica el ORIGEN del frame ENU (lat/lon) a MQTT.

rtk.py fija el origen con el primer fix GPS (donde arranca el robot) y lo publica
retenido. El controller lo consume (MqttOriginInput) para convertir los waypoints
lat/lon al MISMO frame que /odom_filtered. Reemplaza la BASE que antes marcaba el
dashboard: ahora el origen es automatico (el punto de arranque).

Es un output de la capa transport (adapters/mqtt/outputs/): odometry/rtk.py no
importa paho, publica via esta clase inyectada.
"""

import json

from buey_robot.adapters.mqtt.client import MqttClient
from buey_robot.utils.config import require_key


class MqttOriginOutput:
    def __init__(self, client: MqttClient, mqtt_cfg: dict):
        self._client = client
        self.topic = require_key(mqtt_cfg, 'topics', 'odom_origin')

    def send(self, lat: float, lon: float):
        """Publica {lat, lon} del origen ENU (retained: es fijo por corrida)."""
        payload = json.dumps({'lat': round(lat, 8), 'lon': round(lon, 8)})
        self._client.publish(self.topic, payload, retain=True)
