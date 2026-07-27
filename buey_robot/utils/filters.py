"""Filtro de media movil para suavizar mediciones ruidosas."""


class MovingAverageFilter:
    def __init__(self, window_size: int = 5):
        self.window_size = window_size
        self.values = []

    def update(self, value: float) -> float:
        self.values.append(value)
        if len(self.values) > self.window_size:
            self.values.pop(0)
        return sum(self.values) / len(self.values)

    def reset(self):
        self.values = []
