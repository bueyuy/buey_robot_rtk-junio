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
        """Publica string de estado, RETAINED.

        Retained: el broker guarda el ultimo estado y lo entrega a cualquier
        suscriptor apenas se conecta. Asi el dashboard ve el estado actual (p.ej.
        "Ruta cargada — esperando GO") aunque se conecte/reconecte despues, sin
        depender de haber estado escuchando en el instante exacto del cambio.

        Args:
            status_str: String de estado del controlador.
        """
        self._client.publish(self.topic_status, status_str, retain=True)
