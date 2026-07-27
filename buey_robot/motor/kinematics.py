"""Cinematica diferencial skid-steer: (v, w) -> velocidades de rueda izq/der [-100, 100]."""


def differential_to_motor(v: float, w: float, L: float) -> tuple:
    velL = v - w * (L / 2)
    velR = v + w * (L / 2)
    return -velL * 100, -velR * 100   # invertido y escalado al rango del motor
