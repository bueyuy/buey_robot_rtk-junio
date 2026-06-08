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
    """Limita el cambio maximo entre ciclos. Por ahora pass-through."""

    def __init__(self, slew_rate: float = 10.0, slew_rate_stop: float = 20.0):
        self.slew_rate = slew_rate
        self.slew_rate_stop = slew_rate_stop

    def apply(self, value: float) -> float:
        return value
