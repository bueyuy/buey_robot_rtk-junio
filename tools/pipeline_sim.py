"""Pipeline simulation: rectangulo 5x2m pasando por el pipeline completo.

A diferencia de motor_profile.py (que usa valores de motor hardcodeados),
este script pasa por el pipeline real: controller -> gains -> kinematics -> clamp.

Verifica que el pipeline produce los mismos valores de motor que motor_profile.

Uso:
    python3 tools/pipeline_sim.py              # simular + comparacion terminal + JSON
    python3 tools/pipeline_sim.py --upload     # + subir a web
    python3 tools/pipeline_sim.py --plot       # + grafico matplotlib
"""

import argparse
import json
import math
import random
import subprocess
import sys
import os
from datetime import datetime, timezone

# Agregar el paquete al path para importar modulos del proyecto
_pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _pkg_dir)

from buey_robot.motor.kinematics import differential_to_motor
from buey_robot.navigation.ramp_profile import RampProfile
from buey_robot.core.math_utils import angle_diff

# --- Config (mismos valores que navigation.yaml y motor.yaml) ---

# Navigation
CRUISE_LINEAR = 0.3       # m/s
CRUISE_ANGULAR = 0.4      # rad/s
MAX_ANGULAR = 0.6         # rad/s
MIN_LINEAR = 0.05         # m/s
ANGULAR_GAIN = 0.8        # proportional gain (controller)
DECEL_DISTANCE = 2.0      # m
GOAL_TOLERANCE = 0.50     # m
ALIGNMENT_TOLERANCE = math.radians(10.0)  # rad
RAMP_ACCEL_LINEAR = 0.03
RAMP_DECEL_LINEAR = 0.03
RAMP_ACCEL_ANGULAR = 0.03
RAMP_DECEL_ANGULAR = 0.03

# Arc
ARC_LINEAR = 0.233        # m/s
ARC_ANGULAR = 0.302       # rad/s
ARC_START_DISTANCE = 1.5  # m

# Motor
LINEAR_GAIN = 1.2
ANGULAR_GAIN_MOTOR = 2.8
WHEEL_SEPARATION = 0.52
MAX_OUTPUT = 70

# Simulation
DT = 0.1   # 10 Hz
FREQ = 10.0

# Noise (std dev por step a 10Hz, valores tipicos de ZED X visual odometry)
NOISE_HEADING_STD = math.radians(0.5)  # 0.5 deg/step drift heading
NOISE_POSITION_STD = 0.005             # 5mm/step ruido posicion

# Waypoints: rectangulo 5x2m (relativo al origen)
WAYPOINTS = [
    (5.0, 0.0),
    (5.0, 2.0),
    (0.0, 2.0),
    (0.0, 0.0),
]

# Referencia: motor_profile values
MOTOR_PROFILE_CRUISE = (-36, -36)
MOTOR_PROFILE_ARC = (-6, -50)

# Telemetria web
RECORDING_API = "http://192.168.90.100:3000/api/recordings"


# --- Motor Pipeline ---

class MotorPipeline:
    """Pipeline: gains -> kinematics -> clamp. Replica motor_gateway."""

    def __init__(self, linear_gain=LINEAR_GAIN, angular_gain=ANGULAR_GAIN_MOTOR,
                 wheel_sep=WHEEL_SEPARATION, max_output=MAX_OUTPUT):
        self.linear_gain = linear_gain
        self.angular_gain = angular_gain
        self.wheel_sep = wheel_sep
        self.max_output = max_output

    def forward(self, v, w):
        """cmd_vel -> motor values (velL, velR)."""
        v_scaled = v * self.linear_gain
        w_scaled = w * self.angular_gain
        motL, motR = differential_to_motor(v_scaled, w_scaled, self.wheel_sep)
        motL = max(-self.max_output, min(self.max_output, motL))
        motR = max(-self.max_output, min(self.max_output, motR))
        return round(motL, 1), round(motR, 1)

    def inverse(self, motL, motR):
        """motor values -> (v, w) teorico (reversa de forward)."""
        # Revertir clamp (asumimos no estaba clamped)
        # Revertir kinematics: motL = -(v_s - w_s*L/2)*100, motR = -(v_s + w_s*L/2)*100
        v_scaled = -(motL + motR) / 200.0
        w_scaled = -(motR - motL) / (100.0 * self.wheel_sep)
        # Revertir gains
        v = v_scaled / self.linear_gain if self.linear_gain else 0.0
        w = w_scaled / self.angular_gain if self.angular_gain else 0.0
        return v, w


# --- Arc Controller (simula trajectory_controller en modo arc) ---

class ArcController:
    """Replica _control_arc() del trajectory controller."""

    def __init__(self):
        self.ramp_linear = RampProfile(RAMP_ACCEL_LINEAR, RAMP_DECEL_LINEAR)
        self.ramp_angular = RampProfile(RAMP_ACCEL_ANGULAR, RAMP_DECEL_ANGULAR)
        self.state = 'ALIGN'
        self._final_aligning = False

    def step(self, x, y, heading, wp, next_wp):
        """Computa cmd_vel para un step.

        Args:
            x, y, heading: posicion actual del robot.
            wp: (gx, gy) waypoint actual.
            next_wp: (gx, gy) siguiente waypoint, o None.

        Returns:
            (v, w, state, advance): velocidades, estado, y si avanzar WP.
        """
        gx, gy = wp
        dx = gx - x
        dy = gy - y
        distance = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)
        heading_error = angle_diff(angle_to_goal, heading)

        # --- ALIGN ---
        if self.state == 'ALIGN':
            self.ramp_linear.reset()
            if abs(heading_error) < ALIGNMENT_TOLERANCE:
                self.state = 'CRUISE'
            else:
                w = math.copysign(CRUISE_ANGULAR, heading_error)
                return 0.0, w, 'ALIGN', False

        # --- CRUISE ---
        if self.state == 'CRUISE':
            if distance < GOAL_TOLERANCE:
                self.ramp_linear.reset()
                self.ramp_angular.reset()
                self.state = 'ALIGN'
                return 0.0, 0.0, 'CRUISE', True

            if distance < ARC_START_DISTANCE:
                if next_wp is not None:
                    self.state = 'ARC'
                else:
                    self.state = 'FINAL_APPROACH'
                    self._final_aligning = False

            if self.state == 'CRUISE':
                target_vel = CRUISE_LINEAR
                linear_vel = self.ramp_linear.apply(target_vel)
                target_ang = heading_error * ANGULAR_GAIN
                target_ang = max(-MAX_ANGULAR, min(MAX_ANGULAR, target_ang))
                angular_vel = self.ramp_angular.apply(target_ang)
                return linear_vel, angular_vel, 'CRUISE', False

        # --- ARC ---
        if self.state == 'ARC':
            if next_wp is None:
                self.state = 'FINAL_APPROACH'
                self._final_aligning = False
                return 0.0, 0.0, 'ARC', False

            next_dx = next_wp[0] - x
            next_dy = next_wp[1] - y
            angle_to_next = math.atan2(next_dy, next_dx)
            heading_error_next = angle_diff(angle_to_next, heading)

            if abs(heading_error_next) < ALIGNMENT_TOLERANCE:
                self.ramp_linear.reset()
                self.ramp_angular.reset()
                self.state = 'CRUISE'
                return ARC_LINEAR, math.copysign(ARC_ANGULAR, heading_error_next), 'ARC', True

            w = math.copysign(ARC_ANGULAR, heading_error_next)
            return ARC_LINEAR, w, 'ARC', False

        # --- FINAL_APPROACH ---
        if self.state == 'FINAL_APPROACH':
            if self._final_aligning:
                self.ramp_linear.reset()
                if abs(heading_error) < ALIGNMENT_TOLERANCE:
                    self._final_aligning = False
                else:
                    w = math.copysign(CRUISE_ANGULAR, heading_error)
                    return 0.0, w, 'FINAL_APPROACH', False

            if distance < GOAL_TOLERANCE:
                self.ramp_linear.reset()
                self.ramp_angular.reset()
                self.state = 'ALIGN'
                return 0.0, 0.0, 'FINAL_APPROACH', True

            if abs(heading_error) > ALIGNMENT_TOLERANCE * 2:
                self._final_aligning = True
                return 0.0, 0.0, 'FINAL_APPROACH', False

            if distance < DECEL_DISTANCE:
                target_vel = CRUISE_LINEAR * (distance / DECEL_DISTANCE)
                target_vel = max(target_vel, MIN_LINEAR)
            else:
                target_vel = CRUISE_LINEAR

            linear_vel = self.ramp_linear.apply(target_vel)
            target_ang = heading_error * ANGULAR_GAIN
            target_ang = max(-MAX_ANGULAR, min(MAX_ANGULAR, target_ang))
            angular_vel = self.ramp_angular.apply(target_ang)
            return linear_vel, angular_vel, 'FINAL_APPROACH', False

        return 0.0, 0.0, self.state, False


# --- Simulated Robot ---

class SimRobot:
    """Integra posicion con (v, w) del pipeline inverse."""

    def __init__(self, x=0.0, y=0.0, heading=0.0, noise=False):
        self.x = x
        self.y = y
        self.heading = heading
        self.noise = noise

    def step(self, v, w, dt=DT):
        """Actualiza posicion con cinematica diferencial + ruido opcional."""
        self.x += v * math.cos(self.heading) * dt
        self.y += v * math.sin(self.heading) * dt
        self.heading += w * dt

        if self.noise:
            self.x += random.gauss(0, NOISE_POSITION_STD)
            self.y += random.gauss(0, NOISE_POSITION_STD)
            self.heading += random.gauss(0, NOISE_HEADING_STD)

        # Normalizar heading
        while self.heading > math.pi:
            self.heading -= 2 * math.pi
        while self.heading < -math.pi:
            self.heading += 2 * math.pi


# --- Simulation Loop ---

def simulate(noise=False):
    """Ejecuta la simulacion completa del rectangulo 5x2m."""
    pipeline = MotorPipeline()
    controller = ArcController()
    robot = SimRobot(noise=noise)

    waypoints = list(WAYPOINTS)
    wp_idx = 0
    steps = []
    t = 0.0
    max_steps = 2000  # safety: ~200 segundos

    # Recolectar valores por estado para comparacion
    state_motors = {}

    for _ in range(max_steps):
        if wp_idx >= len(waypoints):
            break

        wp = waypoints[wp_idx]
        next_wp = waypoints[wp_idx + 1] if wp_idx + 1 < len(waypoints) else None

        # Controller step
        v_cmd, w_cmd, state, advance = controller.step(
            robot.x, robot.y, robot.heading, wp, next_wp)

        # Pipeline forward
        motL, motR = pipeline.forward(v_cmd, w_cmd)

        # Pipeline inverse (para integracion)
        v_actual, w_actual = pipeline.inverse(motL, motR)

        # Registrar step
        steps.append({
            't': round(t, 3),
            'x': round(robot.x, 4),
            'y': round(robot.y, 4),
            'yaw': round(robot.heading, 4),
            'v_cmd': round(v_cmd, 4),
            'w_cmd': round(w_cmd, 4),
            'motL': motL,
            'motR': motR,
            'state': state,
            'wp_idx': wp_idx,
        })

        # Recolectar para comparacion (solo steady state, ignorar transiciones)
        if state not in state_motors:
            state_motors[state] = []
        state_motors[state].append((motL, motR, v_cmd, w_cmd))

        # Integrar con velocidad del pipeline inverse
        robot.step(v_actual, w_actual)

        # Avanzar WP si controller lo indica
        if advance:
            wp_idx += 1

        t += DT

    return steps, state_motors


# --- Comparacion con motor_profile ---

def print_comparison(state_motors):
    """Compara valores de pipeline_sim vs motor_profile."""
    print("\n=== COMPARACION: pipeline_sim vs motor_profile ===\n")

    # Cruise: buscar steps estables (v=0.3, w~0)
    if 'CRUISE' in state_motors:
        cruise_samples = state_motors['CRUISE']
        # Filtrar solo los que estan a velocidad de crucero (rampa completada)
        stable = [(mL, mR) for mL, mR, v, w in cruise_samples
                  if abs(v - CRUISE_LINEAR) < 0.01]
        if stable:
            avg_motL = sum(s[0] for s in stable) / len(stable)
            avg_motR = sum(s[1] for s in stable) / len(stable)
            min_motL = min(s[0] for s in stable)
            max_motL = max(s[0] for s in stable)
            min_motR = min(s[1] for s in stable)
            max_motR = max(s[1] for s in stable)
            ref_L, ref_R = MOTOR_PROFILE_CRUISE
            print(f"CRUISE: avg motL={avg_motL:.1f} motR={avg_motR:.1f}"
                  f" vs motor_profile ({ref_L:.0f}, {ref_R:.0f})")
            print(f"        range motL=[{min_motL:.1f}, {max_motL:.1f}]"
                  f" motR=[{min_motR:.1f}, {max_motR:.1f}]"
                  f" ({len(stable)} samples)")
        else:
            print("CRUISE: no se encontraron samples estables")

    # Arc: buscar steps en ARC state
    if 'ARC' in state_motors:
        arc_samples = state_motors['ARC']
        # Filtrar los que tienen v=arc_linear (no transiciones)
        stable = [(mL, mR, v, w) for mL, mR, v, w in arc_samples
                  if abs(v - ARC_LINEAR) < 0.01]
        if stable:
            # Separar por signo de w (giro izq vs der)
            left_turns = [(mL, mR) for mL, mR, v, w in stable if w > 0]
            right_turns = [(mL, mR) for mL, mR, v, w in stable if w < 0]

            ref_L, ref_R = MOTOR_PROFILE_ARC  # (-6, -50) = left turn (w > 0)

            for label, samples, eL, eR in [
                ('ARC(L)', left_turns, ref_L, ref_R),
                ('ARC(R)', right_turns, ref_R, ref_L),
            ]:
                if not samples:
                    continue
                avg_motL = sum(s[0] for s in samples) / len(samples)
                avg_motR = sum(s[1] for s in samples) / len(samples)
                min_motL = min(s[0] for s in samples)
                max_motL = max(s[0] for s in samples)
                min_motR = min(s[1] for s in samples)
                max_motR = max(s[1] for s in samples)
                print(f"{label}: avg motL={avg_motL:.1f} motR={avg_motR:.1f}"
                      f" vs motor_profile ({eL:.0f}, {eR:.0f})")
                print(f"        range motL=[{min_motL:.1f}, {max_motL:.1f}]"
                      f" motR=[{min_motR:.1f}, {max_motR:.1f}]"
                      f" ({len(samples)} samples)")
        else:
            print("ARC: no se encontraron samples estables")

    # Verificar constraint: ambas ruedas forward en ARC (motL y motR negativos)
    if 'ARC' in state_motors:
        arc_samples = state_motors['ARC']
        violations = [(mL, mR) for mL, mR, v, w in arc_samples if mL > 0 or mR > 0]
        if violations:
            print(f"\nWARN: {len(violations)} samples con motor positivo en ARC!")
            for mL, mR in violations[:3]:
                print(f"  motL={mL:.1f}, motR={mR:.1f}")
        else:
            print(f"\nConstraint OK: ambas ruedas forward en todos los {len(arc_samples)} ARC samples")

    print()


# --- Print resumen ---

def print_summary(steps):
    """Imprime resumen de la simulacion."""
    total_time = steps[-1]['t']
    xs = [s['x'] for s in steps]
    ys = [s['y'] for s in steps]
    last = steps[-1]

    print(f"=== PIPELINE SIM: Rectangulo 5x2m (arc mode) ===")
    print(f"Total: {total_time:.1f}s | Steps: {len(steps)} | dt={DT}s")
    print(f"Range X: [{min(xs):.2f}, {max(xs):.2f}] Y: [{min(ys):.2f}, {max(ys):.2f}]")
    print(f"Final: ({last['x']:.3f}, {last['y']:.3f}) yaw={math.degrees(last['yaw']):.1f} deg")
    print()

    # Conteo por estado
    from collections import Counter
    state_counts = Counter(s['state'] for s in steps)
    print("Steps por estado:")
    for state, count in sorted(state_counts.items()):
        duration = count * DT
        print(f"  {state:<16} {count:4d} steps ({duration:.1f}s)")
    print()


# --- Plot ---

def plot_path(steps, output_file="tools/pipeline_sim.png"):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print("WARN: matplotlib no disponible, skip plot")
        return

    xs = [s['x'] for s in steps]
    ys = [s['y'] for s in steps]

    # Colores por estado
    state_colors = {
        'ALIGN': 'orange',
        'CRUISE': 'blue',
        'ARC': 'red',
        'FINAL_APPROACH': 'green',
    }

    fig, axes = plt.subplots(1, 3, figsize=(18, 6))

    # Path XY coloreado por estado
    ax = axes[0]
    prev_state = None
    seg_x, seg_y = [], []
    for s in steps:
        if s['state'] != prev_state and seg_x:
            color = state_colors.get(prev_state, 'gray')
            ax.plot(seg_x, seg_y, '-', color=color, linewidth=1.5, label=prev_state)
            seg_x, seg_y = [seg_x[-1]], [seg_y[-1]]
        seg_x.append(s['x'])
        seg_y.append(s['y'])
        prev_state = s['state']
    if seg_x:
        color = state_colors.get(prev_state, 'gray')
        ax.plot(seg_x, seg_y, '-', color=color, linewidth=1.5, label=prev_state)

    # Waypoints
    for i, (wx, wy) in enumerate(WAYPOINTS):
        ax.plot(wx, wy, 'k^', markersize=8)
        ax.annotate(f'WP{i}', (wx, wy), textcoords="offset points", xytext=(5, 5))
    ax.plot(xs[0], ys[0], 'go', markersize=10, label='Start')
    ax.plot(xs[-1], ys[-1], 'rs', markersize=10, label='End')

    # Leyenda sin duplicados
    handles, labels = ax.get_legend_handles_labels()
    seen = set()
    unique = [(h, l) for h, l in zip(handles, labels) if l not in seen and not seen.add(l)]
    ax.legend(*zip(*unique), loc='upper left', fontsize=8)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Trayectoria pipeline_sim - Rectangulo 5x2m')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    # Motor values
    ax2 = axes[1]
    ts = [s['t'] for s in steps]
    motLs = [s['motL'] for s in steps]
    motRs = [s['motR'] for s in steps]
    ax2.plot(ts, motLs, 'r-', label='Motor L', alpha=0.8)
    ax2.plot(ts, motRs, 'b-', label='Motor R', alpha=0.8)
    ax2.axhline(y=MOTOR_PROFILE_CRUISE[0], color='r', linestyle='--', alpha=0.3, label=f'ref cruise {MOTOR_PROFILE_CRUISE[0]}')
    ax2.axhline(y=MOTOR_PROFILE_ARC[0], color='g', linestyle='--', alpha=0.3, label=f'ref arc {MOTOR_PROFILE_ARC[0]}')
    ax2.axhline(y=MOTOR_PROFILE_ARC[1], color='g', linestyle=':', alpha=0.3, label=f'ref arc {MOTOR_PROFILE_ARC[1]}')
    ax2.set_xlabel('Tiempo (s)')
    ax2.set_ylabel('Motor value')
    ax2.set_title('Comandos de motor')
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=7)

    # cmd_vel
    ax3 = axes[2]
    vs = [s['v_cmd'] for s in steps]
    ws = [s['w_cmd'] for s in steps]
    ax3.plot(ts, vs, 'b-', label='v (m/s)', alpha=0.8)
    ax3.plot(ts, ws, 'r-', label='w (rad/s)', alpha=0.8)
    ax3.set_xlabel('Tiempo (s)')
    ax3.set_ylabel('Velocidad')
    ax3.set_title('cmd_vel')
    ax3.grid(True, alpha=0.3)
    ax3.legend()

    plt.tight_layout()
    plt.savefig(output_file, dpi=150)
    print(f"Plot guardado: {output_file}")


# --- JSON export (compatible con telemetria web) ---

def build_json(steps):
    """Genera JSON compatible con motor_profile.build_json()."""
    now = datetime.now(timezone.utc)
    total_duration = steps[-1]['t']
    end_time_dt = datetime.fromtimestamp(now.timestamp() + total_duration, tz=timezone.utc)
    t0_ms = int(now.timestamp() * 1000)

    samples = []
    motor_samples = []
    for s in steps:
        t_ms = t0_ms + int(s['t'] * 1000)
        samples.append({
            "t": t_ms,
            "odom": {
                "x": s['x'],
                "y": s['y'],
                "yaw": s['yaw'],
                "vl": s['v_cmd'],
                "va": s['w_cmd'],
            },
            "cmd": {
                "l": s['v_cmd'],
                "a": s['w_cmd'],
            },
        })
        motor_samples.append({
            "t": t_ms,
            "left": s['motL'],
            "right": s['motR'],
        })

    waypoints_xy = [{"x": wx, "y": wy} for wx, wy in WAYPOINTS]

    return {
        "startTime": now.strftime("%Y-%m-%dT%H:%M:%S.000Z"),
        "endTime": end_time_dt.strftime("%Y-%m-%dT%H:%M:%S.000Z"),
        "samples": samples,
        "motorSamples": motor_samples,
        "waypointsXY": waypoints_xy,
        "params": {"motor": None, "nav2": None},
        "tracking": None,
        "logs": [],
        "metadata": {
            "hostname": "pipeline_sim",
            "testNote": "pipeline sim - rectangulo 5x2m arc mode",
            "controller_mode": "arc",
            "arc_linear": ARC_LINEAR,
            "arc_angular": ARC_ANGULAR,
        },
    }


def save_json(data, path="tools/pipeline_sim.json"):
    with open(path, 'w') as f:
        json.dump(data, f)
    print(f"JSON guardado: {path} ({len(data['samples'])} samples)")


def upload_json(data):
    """POST JSON a la API de telemetria."""
    tmp_path = "/tmp/pipeline_sim_upload.json"
    with open(tmp_path, 'w') as f:
        json.dump(data, f)
    result = subprocess.run(
        ["curl", "-s", "-X", "POST", RECORDING_API,
         "-H", "Content-Type: application/json",
         "-d", f"@{tmp_path}"],
        capture_output=True, text=True,
        timeout=30,
    )
    print(f"Upload response: {result.stdout}")
    if result.returncode != 0 or result.stderr:
        print(f"Upload error: {result.stderr}", file=sys.stderr)


# --- Main ---

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Pipeline simulation: rectangulo 5x2m con arc mode')
    parser.add_argument('--upload', action='store_true',
                        help='Subir JSON a telemetria web')
    parser.add_argument('--plot', action='store_true',
                        help='Generar grafico matplotlib')
    parser.add_argument('--no-json', action='store_true',
                        help='No guardar JSON')
    parser.add_argument('--noise', action='store_true',
                        help='Agregar ruido de visual odometry (heading + posicion)')
    args = parser.parse_args()

    # Verificar pipeline con valores conocidos
    pipe = MotorPipeline()
    cruise_check = pipe.forward(0.3, 0.0)
    arc_check = pipe.forward(0.233, 0.302)
    print("=== Verificacion pipeline ===")
    print(f"cmd_vel(0.3, 0.0)    -> motors {cruise_check}  esperado {MOTOR_PROFILE_CRUISE}")
    print(f"cmd_vel(0.233, 0.302) -> motors {arc_check}  esperado {MOTOR_PROFILE_ARC}")
    print()

    # Simular
    if args.noise:
        print(f"NOISE: heading_std={math.degrees(NOISE_HEADING_STD):.1f} deg/step, "
              f"position_std={NOISE_POSITION_STD*1000:.0f}mm/step\n")
    steps, state_motors = simulate(noise=args.noise)

    # Resumen
    print_summary(steps)
    print_comparison(state_motors)

    # JSON
    if not args.no_json:
        data = build_json(steps)
        save_json(data)
        if args.upload:
            upload_json(data)

    # Plot
    if args.plot:
        plot_path(steps)
