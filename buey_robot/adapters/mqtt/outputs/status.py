"""MqttStatusOutput: publica estado de navegacion por MQTT.

Clase reutilizable (no nodo). TrajectoryController la instancia y llama .send().
Topic publicado:
  - bueyuy/navigation/status (string de estado)
"""

from buey_robot.adapters.mqtt.client import MqttClient
from buey_robot.utils.config import require_key


class MqttStatusOutput:
    """Publica estado de navegacion por MQTT."""

    def __init__(self, client: MqttClient, mqtt_cfg: dict):
        """
        Args:
            client:   Instancia compartida de MqttClient (de get_client()).
            mqtt_cfg: Diccionario de mqtt.yaml.
        """
        self._client = client
        self.topic_status = require_key(mqtt_cfg, 'topics', 'trajectory_status')

    def send(self, status_str: str):
        """Publica string de estado.

        Formato: "Following WP n/m, dist=Xm, heading_err=Y"

        Args:
            status_str: String de estado del controlador.
        """
        self._client.publish(self.topic_status, status_str)
