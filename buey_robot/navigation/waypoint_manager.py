"""Gestion de waypoints: carga, validacion, offset, avance."""

import yaml


class WaypointManager:
    """Carga y administra una lista de waypoints locales (x, y)."""

    def __init__(self):
        self._waypoints = []
        self._index = 0

    def load_from_file(self, filepath: str, origin_x: float, origin_y: float) -> list:
        """Carga waypoints desde un archivo YAML.

        Los waypoints del YAML son relativos al punto de arranque del robot.
        Se suma origin_x/origin_y como offset.

        Crash si el archivo es invalido o no contiene waypoints.

        Args:
            filepath: Ruta al archivo YAML.
            origin_x: Posicion X actual del robot (offset).
            origin_y: Posicion Y actual del robot (offset).

        Returns:
            Lista de tuplas (x, y) con waypoints cargados.
        """
        with open(filepath, 'r') as f:
            data = yaml.safe_load(f)

        if not data or 'waypoints' not in data:
            raise ValueError(f"YAML '{filepath}' no contiene campo 'waypoints'")

        self._waypoints = []
        for wp in data['waypoints']:
            if 'x' not in wp or 'y' not in wp:
                raise ValueError(f"Waypoint invalido (falta x o y): {wp}")
            self._waypoints.append((
                float(wp['x']) + origin_x,
                float(wp['y']) + origin_y,
            ))

        if not self._waypoints:
            raise ValueError("No se encontraron waypoints validos")

        self._index = 0
        return list(self._waypoints)

    def set_waypoints(self, waypoints: list) -> list:
        """Carga waypoints en memoria (coords locales absolutas, sin offset).

        Usado cuando la meta no viene de un archivo (ej: START via MQTT, ya
        convertido a x/y en el frame activo).

        Args:
            waypoints: Lista de tuplas (x, y).

        Returns:
            Lista de waypoints cargados.
        """
        if not waypoints:
            raise ValueError("set_waypoints recibio una lista vacia")
        self._waypoints = [(float(x), float(y)) for x, y in waypoints]
        self._index = 0
        return list(self._waypoints)

    def advance(self):
        """Avanza al siguiente waypoint."""
        self._index += 1

    def restart(self):
        """Vuelve al primer waypoint (para recorrer la ruta en loop)."""
        self._index = 0

    def current_goal(self) -> tuple:
        """Retorna (x, y) del waypoint actual, o None si se completaron todos."""
        if self._index >= len(self._waypoints):
            return None
        return self._waypoints[self._index]

    def peek_next_goal(self) -> tuple:
        """Retorna (x, y) del siguiente waypoint (para futuro mini_arc), o None."""
        next_idx = self._index + 1
        if next_idx >= len(self._waypoints):
            return None
        return self._waypoints[next_idx]

    def is_complete(self) -> bool:
        return self._index >= len(self._waypoints)

    def progress_string(self) -> str:
        return f"{self._index + 1}/{len(self._waypoints)}"

    @property
    def index(self) -> int:
        return self._index

    @property
    def waypoints(self) -> list:
        return list(self._waypoints)

    @property
    def total(self) -> int:
        return len(self._waypoints)
