"""MqttMotorOutput: salida de motores por MQTT. La inyecta MotorGateway (motor/ no
importa paho). Publica bueyuy/navigation/motors ("velL&velR") y cmd_vel de debug."""

from buey_robot.adapters.mqtt.client import MqttClient
from buey_robot.utils.config import require_key


class MqttMotorOutput:
    def __init__(self, client: MqttClient, mqtt_cfg: dict):
        self._client = client
        self.topic_motors = require_key(mqtt_cfg, 'topics', 'motors')
        self.topic_cmd_vel = require_key(mqtt_cfg, 'topics', 'cmd_vel_debug')

    def send(self, velL: float, velR: float, v: float = 0.0, w: float = 0.0):
        self._client.publish(self.topic_motors, f"{velL:.2f}&{velR:.2f}")
        if v != 0.0 or w != 0.0:
            self._client.publish(self.topic_cmd_vel, f"{v:.3f}&{w:.3f}")
