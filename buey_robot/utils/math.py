"""Utilidades matematicas para navegacion."""

import math


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Extrae yaw (heading) de un quaternion.

    Returns:
        Yaw en radianes [-pi, pi].
    """
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_normalize(angle: float) -> float:
    """Normaliza un angulo a [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def angle_diff(target: float, current: float) -> float:
    """Calcula la diferencia angular mas corta (target - current), normalizada a [-pi, pi]."""
    return math.atan2(
        math.sin(target - current),
        math.cos(target - current)
    )
