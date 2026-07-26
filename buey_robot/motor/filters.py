"""Filtro de rampa para el pipeline de motor: limita el cambio de velocidad por tick."""


class SlewRateLimiter:
    """Limita el cambio maximo por tick. accel_rate alejandose de 0, decel_rate acercandose."""

    def __init__(self, accel_rate: float, decel_rate: float):
        self.accel_rate = accel_rate
        self.decel_rate = decel_rate
        self._value = 0.0

    def set_rates(self, accel_rate: float, decel_rate: float):
        self.accel_rate = accel_rate
        self.decel_rate = decel_rate

    def apply(self, target: float) -> float:
        rate = self.accel_rate if abs(target) >= abs(self._value) else self.decel_rate
        delta = max(-rate, min(rate, target - self._value))
        self._value += delta
        return self._value
