"""Carga de YAML de hardware (mqtt.yaml): config que no es parametro ROS2."""

import os
import yaml
from ament_index_python.packages import get_package_share_directory


def load_config(filename: str) -> dict:
    """Carga config/<filename> (dev o instalado). Crash si falta: es error de despliegue."""
    paths = [
        os.path.join(os.path.dirname(__file__), '..', '..', 'config', filename),
        os.path.join(get_package_share_directory('buey_robot'), 'config', filename),
    ]
    for path in paths:
        if os.path.exists(path):
            with open(path, 'r') as f:
                return yaml.safe_load(f)
    raise FileNotFoundError(f"Config '{filename}' no encontrado en {[os.path.abspath(p) for p in paths]}")


def require_key(config: dict, *keys: str):
    """Navega un dict anidado por *keys; crash con el path completo si falta una clave."""
    current = config
    for i, key in enumerate(keys):
        if not isinstance(current, dict) or key not in current:
            raise KeyError(f"Config key '{'.'.join(keys[:i + 1])}' no encontrada")
        current = current[key]
    return current
