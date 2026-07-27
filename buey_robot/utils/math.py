"""Utilidades matematicas para navegacion."""

import math


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Yaw (rad, [-pi, pi]) de un quaternion."""
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def angle_diff(target: float, current: float) -> float:
    """Diferencia angular mas corta (target - current), normalizada a [-pi, pi]."""
    return math.atan2(math.sin(target - current), math.cos(target - current))


def wrap180(deg: float) -> float:
    """Normaliza grados a (-180, 180]."""
    return (deg + 180.0) % 360.0 - 180.0
