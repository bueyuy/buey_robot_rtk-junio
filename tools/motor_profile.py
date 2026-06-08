"""Motor profile: rectangulo 5x2m con valores de motor hardcodeados.

Simula cinematica diferencial, grafica path, y opcionalmente envia por MQTT.

Uso:
    # Simular y graficar
    python3 tools/motor_profile.py --simulate

    # Simular, graficar y subir JSON a telemetria web
    python3 tools/motor_profile.py --simulate --upload

    # Enviar al robot fisico por MQTT (motor_gateway NO debe estar corriendo)
    python3 tools/motor_profile.py --send
"""

import argparse
import json
import math
import subprocess
import sys
import time
from datetime import datetime, timezone

# --- Velocidades empiricas calibradas con test fisico 2026-02-24 ---
# (skid-steer tiene mucha resistencia lateral, formula diferencial no aplica)

# mot(L, R) -> (v m/s, w rad/s) empiricos REALES
EMPIRICAL = {
    (-36, -36): (0.300, 0.000),   # cruise recto
    (-20, -20): (0.167, 0.000),   # decel (mitad de cruise)
    (-6,  -50): (0.163, 0.134),   # arco calibrado test 02-24: w=0.134, v=0.163
    (0,     0): (0.000, 0.000),   # parado
}

DT = 0.1                  # 10 Hz
RAMP_DURATION = 0.5       # transicion suave entre fases (segundos)
DECEL_DURATION = 1.0      # desaceleracion antes del arco

# MQTT
MQTT_BROKER = "192.168.90.24"
MQTT_PORT = 1883
MQTT_TOPIC = "bueyuy/navigation/motors"

# Telemetria web
RECORDING_API = "http://192.168.90.100:3000/api/recordings"


# --- Fases del rectangulo ---

# Comandos de motor
CRUISE = (-36, -36)
DECEL  = (-20, -20)   # velocidad reducida antes del arco
ARC    = (-6, -50)
STOP   = (0, 0)


def simulate_turn():
    """Simula un giro completo (trans_in + arco + trans_out) y retorna
    desplazamiento (dx_along, dy_perp) respecto al heading de entrada,
    mas la duracion del arco puro."""
    v_arc, w_arc = EMPIRICAL[ARC]

    # Calcular duracion de arco para que total = 90 grados
    # trans_in: decel -> arc (w ramps 0 to w_arc)
    # trans_out: arc -> cruise (w ramps w_arc to 0)
    n_ramp = max(1, int(RAMP_DURATION / DT))
    yaw_per_ramp = sum(
        lerp(0, w_arc, i / max(1, n_ramp - 1)) * DT
        for i in range(n_ramp)
    )
    arc_yaw_needed = math.pi / 2.0 - 2 * yaw_per_ramp
    arc_dur = arc_yaw_needed / w_arc

    # Simular el giro completo midiendo desplazamiento
    x, y, yaw = 0.0, 0.0, 0.0
    phases = [
        (DECEL, ARC, RAMP_DURATION),      # trans_in (desde decel, mas lento)
        (ARC, ARC, arc_dur),               # arco
        (ARC, CRUISE, RAMP_DURATION),      # trans_out (accel a cruise)
    ]
    for mot_start, mot_end, dur in phases:
        v_s, w_s = EMPIRICAL[mot_start]
        v_e, w_e = EMPIRICAL[mot_end]
        n = max(1, int(dur / DT))
        for i in range(n):
            frac = i / max(1, n - 1) if n > 1 else 1.0
            v = lerp(v_s, v_e, frac)
            w = lerp(w_s, w_e, frac)
            x += v * math.cos(yaw) * DT
            y += v * math.sin(yaw) * DT
            yaw += w * DT

    # dx_along = avance en heading original, dy_perp = desplazamiento perpendicular
    return x, y, round(arc_dur, 2)


# Dimensiones del rectangulo deseado (centro a centro de waypoints)
RECT_X = 5.0  # metros
RECT_Y = 2.0  # metros


def build_phases_rect():
    """Rectangulo completo 5x2m con decel antes de cada arco."""
    turn_dx, turn_dy, ARC_DUR = simulate_turn()
    v_cruise = EMPIRICAL[CRUISE][0]
    v_decel = EMPIRICAL[DECEL][0]
    decel_dist = (v_cruise + v_decel) / 2.0 * DECEL_DURATION
    v_ramp_avg = v_cruise / 2.0
    ramp_dist = v_ramp_avg * 1.0

    tramo1_dur = max(0.5, (RECT_X - ramp_dist - decel_dist - turn_dx) / v_cruise)
    tramo2_dur = max(0.5, (RECT_Y - turn_dy - decel_dist - turn_dx) / v_cruise)
    tramo3_dur = max(0.5, (RECT_X - turn_dy - decel_dist - turn_dx) / v_cruise)
    tramo4_dur = max(0.5, (RECT_Y - turn_dy - decel_dist - ramp_dist) / v_cruise)

    return [
        ("ramp_up",   STOP,   CRUISE, 1.0),
        ("tramo_1",   CRUISE, CRUISE, round(tramo1_dur, 1)),
        ("decel_1",   CRUISE, DECEL,  DECEL_DURATION),
        ("trans_1",   DECEL,  ARC,    RAMP_DURATION),
        ("arco_wp1",  ARC,    ARC,    ARC_DUR),
        ("trans_2",   ARC,    CRUISE, RAMP_DURATION),
        ("tramo_2",   CRUISE, CRUISE, round(tramo2_dur, 1)),
        ("decel_2",   CRUISE, DECEL,  DECEL_DURATION),
        ("trans_3",   DECEL,  ARC,    RAMP_DURATION),
        ("arco_wp2",  ARC,    ARC,    ARC_DUR),
        ("trans_4",   ARC,    CRUISE, RAMP_DURATION),
        ("tramo_3",   CRUISE, CRUISE, round(tramo3_dur, 1)),
        ("decel_3",   CRUISE, DECEL,  DECEL_DURATION),
        ("trans_5",   DECEL,  ARC,    RAMP_DURATION),
        ("arco_wp3",  ARC,    ARC,    ARC_DUR),
        ("trans_6",   ARC,    CRUISE, RAMP_DURATION),
        ("tramo_4",   CRUISE, CRUISE, round(tramo4_dur, 1)),
        ("ramp_down", CRUISE, STOP,   1.0),
    ]


def build_phases_L():
    """Test L: 5m recto + decel + arco 90deg + 3m recto.
    Para validar que el arco da 90 grados exactos."""
    _, _, ARC_DUR = simulate_turn()
    v_cruise = EMPIRICAL[CRUISE][0]

    return [
        ("ramp_up",   STOP,   CRUISE, 1.0),
        ("tramo_1",   CRUISE, CRUISE, round(5.0 / v_cruise, 1)),
        ("decel_1",   CRUISE, DECEL,  DECEL_DURATION),
        ("trans_1",   DECEL,  ARC,    RAMP_DURATION),
        ("arco_wp1",  ARC,    ARC,    ARC_DUR),
        ("trans_2",   ARC,    CRUISE, RAMP_DURATION),
        ("tramo_2",   CRUISE, CRUISE, round(3.0 / v_cruise, 1)),
        ("ramp_down", CRUISE, STOP,   1.0),
    ]


def build_phases(shape='rect'):
    """Selecciona la forma del perfil."""
    if shape == 'L':
        return build_phases_L()
    else:
        return build_phases_rect()


def lerp(a, b, t):
    """Interpolacion lineal entre a y b, t en [0, 1]."""
    return a + (b - a) * t


def get_empirical_vw(mot):
    """Busca (v, w) empirico para un comando de motor conocido."""
    return EMPIRICAL.get(mot, None)


# --- Simulacion cinematica ---

def simulate(shape='rect'):
    """Simula el perfil completo usando velocidades empiricas."""
    phases = build_phases(shape)
    steps = []

    x, y, yaw = 0.0, 0.0, 0.0
    t_global = 0.0

    for name, mot_start, mot_end, duration in phases:
        v_start, w_start = EMPIRICAL[mot_start]
        v_end, w_end = EMPIRICAL[mot_end]

        n_steps = max(1, int(duration / DT))
        for i in range(n_steps):
            frac = i / max(1, n_steps - 1) if n_steps > 1 else 1.0
            motL = lerp(mot_start[0], mot_end[0], frac)
            motR = lerp(mot_start[1], mot_end[1], frac)
            v = lerp(v_start, v_end, frac)
            w = lerp(w_start, w_end, frac)

            # Integrar posicion
            x += v * math.cos(yaw) * DT
            y += v * math.sin(yaw) * DT
            yaw += w * DT

            steps.append({
                't': round(t_global, 3),
                'x': round(x, 4),
                'y': round(y, 4),
                'yaw': round(yaw, 4),
                'v': round(v, 4),
                'w': round(w, 4),
                'motL': round(motL, 1),
                'motR': round(motR, 1),
                'phase': name,
            })
            t_global += DT

    return steps


# --- Graficar ---

def plot_path(steps, output_file="tools/motor_profile.png"):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print("WARN: matplotlib no disponible, skip plot")
        return

    xs = [s['x'] for s in steps]
    ys = [s['y'] for s in steps]

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    # Path XY
    ax = axes[0]
    ax.plot(xs, ys, 'b-', linewidth=1.5)
    ax.plot(xs[0], ys[0], 'go', markersize=10, label='Start')
    ax.plot(xs[-1], ys[-1], 'rs', markersize=10, label='End')

    # Marcar waypoints (esquinas del rectangulo)
    wp_indices = []
    for i, s in enumerate(steps):
        if s['phase'].startswith('arco_') and (i == 0 or steps[i-1]['phase'] != s['phase']):
            wp_indices.append(i)
    for j, idx in enumerate(wp_indices):
        ax.plot(steps[idx]['x'], steps[idx]['y'], 'k^', markersize=8)
        ax.annotate(f'WP{j+1}', (steps[idx]['x'], steps[idx]['y']),
                     textcoords="offset points", xytext=(5, 5))

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Trayectoria simulada - Rectangulo 5x2m')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend()

    # Motor values over time
    ax2 = axes[1]
    ts = [s['t'] for s in steps]
    motLs = [s['motL'] for s in steps]
    motRs = [s['motR'] for s in steps]
    ax2.plot(ts, motLs, 'r-', label='Motor L', alpha=0.8)
    ax2.plot(ts, motRs, 'b-', label='Motor R', alpha=0.8)
    ax2.set_xlabel('Tiempo (s)')
    ax2.set_ylabel('Motor value')
    ax2.set_title('Comandos de motor')
    ax2.grid(True, alpha=0.3)
    ax2.legend()

    plt.tight_layout()
    plt.savefig(output_file, dpi=150)
    print(f"Plot guardado: {output_file}")


# --- Export JSON (compatible con telemetria web) ---

def build_json(steps):
    """Genera JSON con campos principales: samples (x,y) y motorSamples (L,R)."""
    now = datetime.now(timezone.utc)
    total_duration = steps[-1]['t']
    start_time = now
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
                "vl": s['v'],
                "va": s['w'],
            },
            "cmd": {
                "l": s['v'],
                "a": s['w'],
            },
        })
        motor_samples.append({
            "t": t_ms,
            "left": s['motL'],
            "right": s['motR'],
        })

    # Waypoints: esquinas del rectangulo teorico
    waypoints = [
        {"x": 0, "y": 0},
        {"x": 5, "y": 0},
        {"x": 5, "y": 2},
        {"x": 0, "y": 2},
    ]

    return {
        "startTime": start_time.strftime("%Y-%m-%dT%H:%M:%S.000Z"),
        "endTime": end_time_dt.strftime("%Y-%m-%dT%H:%M:%S.000Z"),
        "samples": samples,
        "motorSamples": motor_samples,
        "waypointsXY": waypoints,
        "params": {"motor": None, "nav2": None},
        "tracking": None,
        "logs": [],
        "metadata": {
            "hostname": "motor_profile",
            "testNote": "motor profile simulado - rectangulo 5x2m",
        },
    }


def save_json(data, path="tools/motor_profile_simulated.json"):
    with open(path, 'w') as f:
        json.dump(data, f)
    print(f"JSON guardado: {path} ({len(data['samples'])} samples)")


def upload_json(data):
    """POST JSON a la API de telemetria via archivo temporal."""
    tmp_path = "/tmp/motor_profile_upload.json"
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


# --- Envio MQTT ---

def send_mqtt(steps):
    """Envia motor values al robot en tiempo real via MQTT a 10Hz."""
    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        print("ERROR: paho-mqtt no instalado. pip install paho-mqtt", file=sys.stderr)
        sys.exit(1)

    client = mqtt.Client()
    client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
    client.loop_start()

    total = len(steps)
    total_time = steps[-1]['t']
    print(f"Enviando {total} steps ({total_time:.1f}s) a {MQTT_TOPIC}")
    print("Ctrl+C para cancelar\n")

    try:
        t_start = time.monotonic()
        for i, s in enumerate(steps):
            # Esperar hasta el momento correcto
            target_time = t_start + s['t']
            now = time.monotonic()
            if target_time > now:
                time.sleep(target_time - now)

            payload = f"{s['motL']:.2f}&{s['motR']:.2f}"
            client.publish(MQTT_TOPIC, payload, qos=0)

            if i % 50 == 0:  # print cada 5s
                print(f"  t={s['t']:5.1f}s  mot({s['motL']:5.1f}, {s['motR']:5.1f})  "
                      f"pos=({s['x']:.2f}, {s['y']:.2f})  phase={s['phase']}")

        # Stop al final
        client.publish(MQTT_TOPIC, "0.00&0.00", qos=0)
        print("\nCompletado. Motores en 0.")

    except KeyboardInterrupt:
        client.publish(MQTT_TOPIC, "0.00&0.00", qos=0)
        print("\nCancelado. Motores en 0.")

    finally:
        client.loop_stop()
        client.disconnect()


# --- Print resumen ---

def print_summary(steps):
    """Imprime resumen de la simulacion."""
    total_time = steps[-1]['t']
    xs = [s['x'] for s in steps]
    ys = [s['y'] for s in steps]
    last = steps[-1]

    print(f"=== MOTOR PROFILE: Rectangulo 5x2m ===")
    print(f"Total: {total_time:.1f}s | Steps: {len(steps)} | dt={DT}s")
    print(f"Range X: [{min(xs):.2f}, {max(xs):.2f}] Y: [{min(ys):.2f}, {max(ys):.2f}]")
    print(f"Final: ({last['x']:.3f}, {last['y']:.3f}) yaw={math.degrees(last['yaw']):.1f} deg")
    print()

    # Resumen por fase
    print(f"{'Fase':<12} {'t_start':>7} {'t_end':>7} {'motL':>6} {'motR':>6} {'v':>6} {'w':>6}")
    print("-" * 60)
    current_phase = None
    phase_start = 0
    for s in steps:
        if s['phase'] != current_phase:
            if current_phase:
                print(f"{current_phase:<12} {phase_start:7.1f} {s['t']:7.1f} "
                      f"{prev['motL']:6.1f} {prev['motR']:6.1f} "
                      f"{prev['v']:6.3f} {prev['w']:6.3f}")
            current_phase = s['phase']
            phase_start = s['t']
        prev = s
    # Ultima fase
    print(f"{current_phase:<12} {phase_start:7.1f} {steps[-1]['t']:7.1f} "
          f"{prev['motL']:6.1f} {prev['motR']:6.1f} "
          f"{prev['v']:6.3f} {prev['w']:6.3f}")
    print()


# --- Main ---

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Motor profile: rectangulo 5x2m')
    parser.add_argument('--simulate', action='store_true',
                        help='Simular, graficar y guardar JSON')
    parser.add_argument('--upload', action='store_true',
                        help='Subir JSON simulado a telemetria web')
    parser.add_argument('--send', action='store_true',
                        help='Enviar motor values al robot por MQTT en tiempo real')
    parser.add_argument('--shape', choices=['rect', 'L'], default='rect',
                        help='Forma: rect (rectangulo completo) o L (un arco)')
    parser.add_argument('--no-plot', action='store_true',
                        help='Skip matplotlib plot')
    args = parser.parse_args()

    if not args.simulate and not args.send:
        parser.print_help()
        print("\nEjemplo: python3 tools/motor_profile.py --simulate --shape L")
        sys.exit(1)

    steps = simulate(args.shape)
    print_summary(steps)

    if args.simulate:
        if not args.no_plot:
            plot_path(steps)

        data = build_json(steps)
        save_json(data)

        if args.upload:
            upload_json(data)

    if args.send:
        print("=== MODO ENVIO MQTT ===")
        print(f"Broker: {MQTT_BROKER}:{MQTT_PORT}")
        print(f"Topic: {MQTT_TOPIC}")
        print("ATENCION: motor_gateway NO debe estar corriendo!")
        print()
        confirm = input("Enviar? [y/N] ")
        if confirm.lower() == 'y':
            send_mqtt(steps)
        else:
            print("Cancelado.")
