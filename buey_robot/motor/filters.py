"""Filtros para el pipeline de motor.

Activos:
    - SoftDeadzone: Rampa lineal en zona muerta, elimina snap.

Futuros (documentados, pass-through por ahora):
    - EMAFilter: Suavizado exponencial para picos de cmd_vel.
    - SlewRateLimiter: Limita delta maximo entre ciclos.
"""

import math


class SoftDeadzone:
    """Deadzone suave con rampa lineal. Sin snap brusco.

    Comportamiento:
        |val| < low  -> 0          (zona muerta real)
        |val| in [low, high] -> rampa lineal de 0 a val
        |val| > high -> pass through
    """

    def __init__(self, low: float, high: float):
        if low < 0 or high < 0 or high <= low:
            raise ValueError(f"Requiere 0 <= low < high, got low={low}, high={high}")
        self.low = low
        self.high = high

    def apply(self, value: float) -> float:
        """Aplica la soft deadzone."""
        abs_val = abs(value)
        if abs_val < self.low:
            return 0.0
        if abs_val >= self.high:
            return value
        t = (abs_val - self.low) / (self.high - self.low)
        return math.copysign(t * abs_val, value)


class EMAFilter:
    """Filtro exponencial de media movil. Por ahora pass-through."""

    def __init__(self, alpha: float = 0.3):
        self.alpha = alpha

    def apply(self, value: float) -> float:
        return value


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
