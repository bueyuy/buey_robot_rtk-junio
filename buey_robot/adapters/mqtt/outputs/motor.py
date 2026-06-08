"""MqttMotorOutput: publica velocidades de motor por MQTT.

Clase reutilizable (no nodo). Motor Gateway la instancia y llama .send().
Topics publicados:
  - bueyuy/navigation/motors  (string "velL&velR")
  - bueyuy/navigation/cmd_vel (string "v&w") -- debug
"""

from buey_robot.adapters.mqtt.client import MqttClient
from buey_robot.utils.config import require_key


class MqttMotorOutput:
    """Publica velocidades de motor por MQTT."""

    def __init__(self, client: MqttClient, mqtt_cfg: dict):
        """
        Args:
            client:   Instancia compartida de MqttClient (de get_client()).
            mqtt_cfg: Diccionario de mqtt.yaml.
        """
        self._client = client
        self.topic_motors = require_key(mqtt_cfg, 'topics', 'motors')
        self.topic_cmd_vel = require_key(mqtt_cfg, 'topics', 'cmd_vel_debug')

    def send(self, velL: float, velR: float, v: float = 0.0, w: float = 0.0):
        """Publica velocidades de motor y cmd_vel de debug.

        Args:
            velL: Velocidad rueda izquierda (-100 a 100).
            velR: Velocidad rueda derecha (-100 a 100).
            v: Velocidad lineal original (m/s), para debug.
            w: Velocidad angular original (rad/s), para debug.
        """
        payload = f"{velL:.2f}&{velR:.2f}"
        self._client.publish(self.topic_motors, payload)

        if v != 0.0 or w != 0.0:
            self._client.publish(self.topic_cmd_vel, f"{v:.3f}&{w:.3f}")
