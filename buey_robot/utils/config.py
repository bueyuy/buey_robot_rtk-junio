"""Carga de configuracion YAML para archivos de hardware (mqtt, sensors).

load_config se usa para mqtt.yaml y sensors.yaml — configuracion de hardware
que no es parametro ROS2. Los nodos de navegacion/motor usan declare_parameter
+ get_parameter leyendo desde los YAMLs pasados en parameters=[...] del launch.
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory


def load_config(filename: str) -> dict:
    """Carga un archivo YAML de config/.

    Busca primero en el path de desarrollo y luego en el path instalado.
    Crash si no existe: config faltante = error de despliegue.

    Args:
        filename: Nombre del archivo (ej: 'mqtt.yaml')

    Returns:
        Diccionario con la configuracion.
    """
    possible_paths = [
        # Development path (running from src)
        os.path.join(os.path.dirname(__file__), '..', '..', 'config', filename),
        # Installed path
        os.path.join(get_package_share_directory('buey_robot'), 'config', filename),
    ]

    for path in possible_paths:
        abs_path = os.path.abspath(path)
        if os.path.exists(abs_path):
            with open(abs_path, 'r') as f:
                return yaml.safe_load(f)

    tried = [os.path.abspath(p) for p in possible_paths]
    raise FileNotFoundError(
        f"Config '{filename}' no encontrado. Buscado en:\n"
        + "\n".join(f"  - {p}" for p in tried)
    )


def require_key(config: dict, *keys: str):
    """Navega un diccionario anidado y retorna el valor.

    Crash con path completo si falta alguna clave.

    Args:
        config: Diccionario de configuracion.
        *keys: Secuencia de claves para navegar (ej: 'motor_control', 'wheel_separation')

    Returns:
        Valor encontrado.
    """
    current = config
    path = []
    for key in keys:
        path.append(key)
        if not isinstance(current, dict) or key not in current:
            full_path = '.'.join(path)
            raise KeyError(f"Config key '{full_path}' no encontrada")
        current = current[key]
    return current
