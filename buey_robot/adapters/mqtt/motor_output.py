"""MqttMotorOutput: salida de motores por MQTT. La inyecta MotorGateway (motor/ no importa
paho). Motores a rate de control; cmd_vel es telemetria (el gateway lo throttlea)."""

from buey_robot.adapters.mqtt.client import MqttClient
from buey_robot.utils.config import require_key


class MqttMotorOutput:
    def __init__(self, client: MqttClient, mqtt_cfg: dict):
        self._client = client
        self.topic_motors = require_key(mqtt_cfg, 'topics', 'motors')
        self.topic_cmd_vel = require_key(mqtt_cfg, 'topics', 'cmd_vel_debug')

    def send_motors(self, velL: float, velR: float):
        self._client.publish(self.topic_motors, f"{velL:.2f}&{velR:.2f}")

    def send_cmd_vel(self, v: float, w: float):
        self._client.publish(self.topic_cmd_vel, f"{v:.3f}&{w:.3f}")
