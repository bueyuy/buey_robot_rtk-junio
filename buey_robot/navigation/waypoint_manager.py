"""Gestion de waypoints: carga (via set_waypoints), avance, loop.

Los waypoints llegan SIEMPRE por MQTT (bueyuy/waypoints), ya convertidos a x,y
locales por el controller. No se cargan de archivos YAML.
"""


class WaypointManager:
    """Carga y administra una lista de waypoints locales (x, y)."""

    def __init__(self):
        self._waypoints = []
        self._index = 0

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
