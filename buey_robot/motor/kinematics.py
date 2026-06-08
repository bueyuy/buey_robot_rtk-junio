"""Cinematica diferencial para robot skid-steer.

Funcion pura: misma entrada siempre produce misma salida.
"""


def differential_to_motor(v: float, w: float, L: float) -> tuple:
    """Convierte velocidad lineal y angular a comandos de motor izq/der.

    Pipeline:
        1. Cinematica diferencial: velR = v + w*(L/2), velL = v - w*(L/2)
        2. Escalar a rango motores: -(vel) * 100

    Args:
        v: Velocidad lineal (m/s), ya escalada con gains.
        w: Velocidad angular (rad/s), ya escalada con gains.
        L: Separacion entre ruedas (metros).

    Returns:
        Tupla (velL, velR) en escala -100 a 100.
    """
    velR = v + w * (L / 2)
    velL = v - w * (L / 2)

    velR = -(velR) * 100
    velL = -(velL) * 100

    return velL, velR
