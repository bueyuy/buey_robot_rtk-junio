"""Gestion de una lista de waypoints locales (x, y): carga, avance, loop."""


class WaypointManager:
    def __init__(self):
        self._waypoints = []
        self._index = 0

    def set_waypoints(self, waypoints: list):
        if not waypoints:
            raise ValueError("set_waypoints recibio una lista vacia")
        self._waypoints = [(float(x), float(y)) for x, y in waypoints]
        self._index = 0

    def advance(self):
        self._index += 1

    def restart(self):
        self._index = 0

    def current_goal(self) -> tuple:
        """(x, y) del waypoint actual, o None si se completaron todos."""
        if self._index >= len(self._waypoints):
            return None
        return self._waypoints[self._index]

    def is_complete(self) -> bool:
        return self._index >= len(self._waypoints)

    def progress_string(self) -> str:
        return f"{self._index + 1}/{len(self._waypoints)}"

    @property
    def index(self) -> int:
        return self._index

    @property
    def total(self) -> int:
        return len(self._waypoints)
