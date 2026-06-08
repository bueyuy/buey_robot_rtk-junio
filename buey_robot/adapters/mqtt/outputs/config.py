"""MqttConfigOutput: publica params de configuracion del robot (retained, una vez).

Clase reutilizable (no nodo). TrajectoryController la llama al iniciar.
Topics publicados (retained):
  - bueyuy/config        (params runtime del robot)
  - bueyuy/config/motors (params runtime de motores)
"""

import json

from buey_robot.adapters.mqtt.client import MqttClient
from buey_robot.utils.config import require_key


class MqttConfigOutput:
    """Publica configuracion del robot como retained en MQTT."""

    def __init__(self, client: MqttClient, mqtt_cfg: dict):
        """
        Args:
            client:   Instancia compartida de MqttClient (de get_client()).
            mqtt_cfg: Diccionario de mqtt.yaml.
        """
        self._client = client
        self.topic_config = require_key(mqtt_cfg, 'topics', 'config')
        self.topic_config_motors = require_key(mqtt_cfg, 'topics', 'config_motors')

    def publish_once(self, nav_params: dict, motor_params: dict):
        """Publica configuracion del robot y motores (retained).

        Args:
            nav_params: Dict con campos planos de navegacion:
                        mode, goal_tolerance_m, alignment_tolerance_deg,
                        cruise_linear_m_s, cruise_angular_rad_s, max_angular_rad_s.
            motor_params: Dict con campos planos de motor:
                        wheel_separation, max_output, linear_gain, angular_gain,
                        deadzone_low, deadzone_high.
        """
        robot_config = json.dumps({
            'mode': nav_params['mode'],
            'goal_tolerance_m': nav_params['goal_tolerance_m'],
            'alignment_tolerance_deg': nav_params['alignment_tolerance_deg'],
            'cruise_linear_m_s': nav_params['cruise_linear_m_s'],
            'cruise_angular_rad_s': nav_params['cruise_angular_rad_s'],
            'max_angular_rad_s': nav_params['max_angular_rad_s'],
        })

        motors_config = json.dumps({
            'wheel_separation': motor_params['wheel_separation'],
            'max_output': motor_params['max_output'],
            'linear_gain': motor_params['linear_gain'],
            'angular_gain': motor_params['angular_gain'],
            'deadzone_low': motor_params['deadzone_low'],
            'deadzone_high': motor_params['deadzone_high'],
        })

        self._client.publish(self.topic_config, robot_config, retain=True)
        self._client.publish(self.topic_config_motors, motors_config, retain=True)
