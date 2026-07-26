"""Filtros de senal para suavizado de mediciones."""


class MovingAverageFilter:
    """Filtro de media movil para suavizar mediciones ruidosas."""

    def __init__(self, window_size: int = 5):
        self.window_size = window_size
        self.values = []

    def update(self, value: float) -> float:
        """Agrega un nuevo valor y retorna el promedio filtrado."""
        self.values.append(value)
        if len(self.values) > self.window_size:
            self.values.pop(0)
        return sum(self.values) / len(self.values)

    def reset(self):
        self.values = []
