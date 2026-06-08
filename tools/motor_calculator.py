"""Calculadora de motor pipeline: cmd_vel <-> motor outputs.

Ingenieria inversa del pipeline completo:
  cmd_vel(v, w) -> gains -> differential -> scale -> motor(L, R)

Uso:
    # Forward: cmd_vel -> motors
    python3 tools/motor_calculator.py --forward 0.3 0.0
    python3 tools/motor_calculator.py --forward 0.1 0.124

    # Reverse: motors -> cmd_vel
    python3 tools/motor_calculator.py --reverse 6 46

    # Sweep: tabla de escenarios
    python3 tools/motor_calculator.py --sweep

    # Decel zone: que pasa a distintas distancias del WP
    python3 tools/motor_calculator.py --decel
"""

import argparse

# Motor config (motor.yaml)
LINEAR_GAIN = 1.2
ANGULAR_GAIN = 2.8
WHEEL_SEP = 0.52
MAX_OUTPUT = 70
SOFT_DZ_LOW = 5.0
SOFT_DZ_HIGH = 20.0

# Nav config (navigation.yaml)
CRUISE_LINEAR = 0.3
CRUISE_ANGULAR = 0.4
MIN_LINEAR = 0.1
MAX_ANGULAR = 0.6
ANGULAR_GAIN_NAV = 0.8
DECEL_DISTANCE = 2.0


def cmd_to_motors(v: float, w: float) -> tuple:
    """cmd_vel(v, w) -> motor(L, R)"""
    v_scaled = v * LINEAR_GAIN
    w_scaled = w * ANGULAR_GAIN

    velR_raw = v_scaled + w_scaled * (WHEEL_SEP / 2)
    velL_raw = v_scaled - w_scaled * (WHEEL_SEP / 2)

    motL = -(velL_raw) * 100
    motR = -(velR_raw) * 100

    motL = max(-MAX_OUTPUT, min(MAX_OUTPUT, motL))
    motR = max(-MAX_OUTPUT, min(MAX_OUTPUT, motR))

    return motL, motR


def motors_to_cmd(motL: float, motR: float) -> tuple:
    """motor(L, R) -> cmd_vel(v, w)"""
    velL_raw = -(motL) / 100
    velR_raw = -(motR) / 100

    v_scaled = (velL_raw + velR_raw) / 2
    w_scaled = (velR_raw - velL_raw) / WHEEL_SEP

    v = v_scaled / LINEAR_GAIN
    w = w_scaled / ANGULAR_GAIN

    return v, w


def deadzone_status(mot: float) -> str:
    """Indica si el valor esta en deadzone."""
    m = abs(mot)
    if m < SOFT_DZ_LOW:
        return "DEAD"
    elif m < SOFT_DZ_HIGH:
        return "weak"
    else:
        return "ok"


def print_forward(v, w):
    motL, motR = cmd_to_motors(v, w)
    print(f"cmd_vel: v={v:.3f} m/s, w={w:.3f} rad/s")
    print(f"  gains: v_scaled={v*LINEAR_GAIN:.3f}, w_scaled={w*ANGULAR_GAIN:.3f}")
    print(f"  motors: L={motL:.1f} [{deadzone_status(motL)}], R={motR:.1f} [{deadzone_status(motR)}]")
    print(f"  diff: {abs(motR - motL):.1f}, avg: {(abs(motL) + abs(motR))/2:.1f}")


def print_reverse(motL, motR):
    v, w = motors_to_cmd(motL, motR)
    print(f"motors: L={motL:.1f}, R={motR:.1f}")
    print(f"  cmd_vel: v={v:.3f} m/s, w={w:.3f} rad/s")
    print(f"  heading_error needed (gain={ANGULAR_GAIN_NAV}): {w/ANGULAR_GAIN_NAV:.1f} rad = {(w/ANGULAR_GAIN_NAV)*57.3:.1f} deg")


def print_sweep():
    print("=== SWEEP: cmd_vel -> motors ===")
    print(f"{'v':>6} {'w':>6} {'motL':>6} {'motR':>6} {'dzL':>5} {'dzR':>5} {'scenario'}")
    print("-" * 65)

    scenarios = [
        (0.3, 0.0, "cruise recto"),
        (0.3, 0.1, "cruise + correccion leve"),
        (0.3, 0.2, "cruise + correccion media"),
        (0.3, 0.4, "cruise + giro fuerte"),
        (0.3, 0.6, "cruise + max angular"),
        (0.2, 0.0, "decel medio recto"),
        (0.2, 0.1, "decel medio + correccion"),
        (0.2, 0.2, "decel medio + giro"),
        (0.1, 0.0, "min_linear recto"),
        (0.1, 0.05, "min_linear + correccion minima"),
        (0.1, 0.1, "min_linear + correccion leve"),
        (0.1, 0.124, "min_linear + 10deg*gain (ATASCO recording)"),
        (0.1, 0.2, "min_linear + correccion media"),
        (0.1, 0.3, "min_linear + giro fuerte"),
        (0.1, 0.6, "min_linear + max angular"),
        (0.15, 0.15, "equilibrado medio"),
        (0.15, 0.3, "equilibrado con giro"),
        (0.0, 0.4, "puro giro (alignment)"),
    ]

    for v, w, label in scenarios:
        motL, motR = cmd_to_motors(v, w)
        dzL = deadzone_status(motL)
        dzR = deadzone_status(motR)
        print(f"{v:6.2f} {w:6.2f} {motL:6.1f} {motR:6.1f} {dzL:>5} {dzR:>5}  {label}")


def print_decel():
    print("=== DECEL ZONE: distancia -> velocidad -> motores ===")
    print(f"Config: cruise={CRUISE_LINEAR}, min_linear={MIN_LINEAR}, decel_dist={DECEL_DISTANCE}")
    print()

    heading_errors_deg = [0, 2, 5, 10, 15]

    for he_deg in heading_errors_deg:
        he_rad = he_deg * 3.14159 / 180
        w = min(he_rad * ANGULAR_GAIN_NAV, MAX_ANGULAR)

        print(f"--- heading_error = {he_deg} deg (w={w:.3f} rad/s) ---")
        print(f"{'dist':>5} {'v_target':>8} {'motL':>6} {'motR':>6} {'dzL':>5} {'dzR':>5}")

        for dist_cm in [200, 150, 100, 80, 60, 40, 30, 20]:
            dist = dist_cm / 100
            if dist < DECEL_DISTANCE:
                v_target = CRUISE_LINEAR * (dist / DECEL_DISTANCE)
                v_target = max(v_target, MIN_LINEAR)
            else:
                v_target = CRUISE_LINEAR

            motL, motR = cmd_to_motors(v_target, w)
            dzL = deadzone_status(motL)
            dzR = deadzone_status(motR)
            print(f"{dist:5.2f} {v_target:8.3f} {motL:6.1f} {motR:6.1f} {dzL:>5} {dzR:>5}")
        print()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Motor pipeline calculator')
    parser.add_argument('--forward', nargs=2, type=float, metavar=('V', 'W'),
                        help='cmd_vel(v, w) -> motors')
    parser.add_argument('--reverse', nargs=2, type=float, metavar=('MOT_L', 'MOT_R'),
                        help='motors(L, R) -> cmd_vel')
    parser.add_argument('--sweep', action='store_true', help='Tabla de escenarios')
    parser.add_argument('--decel', action='store_true', help='Decel zone analysis')
    args = parser.parse_args()

    if args.forward:
        print_forward(args.forward[0], args.forward[1])
    elif args.reverse:
        print_reverse(args.reverse[0], args.reverse[1])
    elif args.sweep:
        print_sweep()
    elif args.decel:
        print_decel()
    else:
        print("=== TARGET DEL USUARIO: mot(6, 46) ===")
        print_reverse(-6, -46)
        print()
        print_sweep()
        print()
        print_decel()
