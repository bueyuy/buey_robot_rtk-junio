"""Perfil de rampa asimetrica para aceleracion/desaceleracion suave.

La rampa suaviza cambios de velocidad para suavizar el arranque/frenado (skid-steer).
decel_rate > accel_rate: frena mas rapido de lo que acelera.
"""


class RampProfile:
    """Rampa asimetrica: limita incremento y decremento de velocidad por ciclo."""

    def __init__(self, accel_rate: float, decel_rate: float):
        """
        Args:
            accel_rate: Incremento maximo por ciclo al acelerar (ej: 0.03 m/s a 10Hz = 0.3 m/s²).
            decel_rate: Decremento maximo por ciclo al frenar (ej: 0.06 m/s a 10Hz = 0.6 m/s²).
        """
        self.accel_rate = accel_rate
        self.decel_rate = decel_rate
        self._current = 0.0

    def apply(self, target: float) -> float:
        """Aplica la rampa al valor objetivo.

        Args:
            target: Velocidad deseada.

        Returns:
            Velocidad rampeada (limitada por accel/decel rate).
        """
        diff = target - self._current
        if diff > 0:
            # Acelerando
            if diff > self.accel_rate:
                self._current += self.accel_rate
            else:
                self._current = target
        elif diff < 0:
            # Desacelerando
            if abs(diff) > self.decel_rate:
                self._current -= self.decel_rate
            else:
                self._current = target

        return self._current

    def reset(self):
        """Reset de la rampa (ej: al cambiar de modo alineacion -> avance)."""
        self._current = 0.0

    @property
    def current(self) -> float:
        return self._current
