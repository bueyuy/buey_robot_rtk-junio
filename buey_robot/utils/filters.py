"""Filtros de senal para suavizado de mediciones."""

from typing import Optional


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


class ExponentialFilter:
    """Filtro exponencial simple para suavizado de senales."""

    def __init__(self, alpha: float = 0.3):
        """
        Args:
            alpha: Factor de suavizado (0-1). Valores menores = mas suavizado.
        """
        self.alpha = alpha
        self.value: Optional[float] = None

    def update(self, measurement: float) -> float:
        """Actualiza el filtro con una nueva medicion."""
        if self.value is None:
            self.value = measurement
        else:
            self.value = self.alpha * measurement + (1 - self.alpha) * self.value
        return self.value

    def reset(self):
        self.value = None
