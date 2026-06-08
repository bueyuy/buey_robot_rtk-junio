"""Analisis de recordings de telemetria del Buey robot.

Uso:
    python3 tools/analyze_recording.py                  # ultimo recording
    python3 tools/analyze_recording.py --summary        # solo resumen
    python3 tools/analyze_recording.py --phase 1        # detalle de fase (0-indexed)
    python3 tools/analyze_recording.py --raw 20 40      # samples crudos entre t=20s y t=40s
"""

import json
import math
import argparse
import subprocess
import sys

RECORDING_URL = "http://192.168.90.100:3000/api/recordings/latest"
LOCAL_PATH = "/tmp/recording_latest.json"


def fetch_recording():
    subprocess.run(["curl", "-s", RECORDING_URL, "-o", LOCAL_PATH], check=True)
    with open(LOCAL_PATH) as f:
        return json.load(f)


def dist(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)


def find_closest_motor(motor_samples, t):
    """Binary search for closest motor sample by timestamp."""
    lo, hi = 0, len(motor_samples) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if motor_samples[mid]['t'] < t:
            lo = mid + 1
        else:
            hi = mid
    return motor_samples[lo] if motor_samples else {'left': 0, 'right': 0}


def detect_phases(samples, t0):
    """Detecta fases: idle, aligning, forward, stuck."""
    phases = []
    current_phase = None

    for s in samples:
        t = (s['t'] - t0) / 1000
        cl = s['cmd']['l']
        ca = s['cmd']['a']

        if abs(cl) < 0.01 and abs(ca) < 0.01:
            phase_type = 'idle'
        elif abs(cl) < 0.01 and abs(ca) > 0.05:
            phase_type = 'aligning'
        elif abs(cl) > 0.01 and abs(ca) < 0.05:
            phase_type = 'forward'
        else:
            phase_type = 'mixed'  # linear + angular simultaneo

        if phase_type != current_phase:
            if phases:
                phases[-1]['t_end'] = t
            phases.append({
                'type': phase_type,
                't_start': t,
                't_end': t,
                'odom_start': s['odom'].copy(),
                'cmd_start': s['cmd'].copy(),
            })
            current_phase = phase_type
        else:
            phases[-1]['t_end'] = t
            phases[-1]['odom_end'] = s['odom'].copy()
            phases[-1]['cmd_end'] = s['cmd'].copy()

    return phases


def print_summary(data):
    samples = data['samples']
    ms = data.get('motorSamples', [])
    wps = data.get('waypointsXY', [])
    t0 = samples[0]['t']
    duration = (samples[-1]['t'] - t0) / 1000

    print(f"=== RECORDING: {data['filename']} ===")
    print(f"Duration: {duration:.1f}s | Samples: {len(samples)} | Motor: {len(ms)}")
    if data.get('metadata', {}).get('testNote'):
        print(f"Note: {data['metadata']['testNote']}")
    print()

    # Waypoints
    print("=== WAYPOINTS ===")
    for i, wp in enumerate(wps):
        print(f"  WP{i}: ({wp['x']:.2f}, {wp['y']:.2f})")
    print()

    # Waypoint arrivals
    print("=== WAYPOINT PROXIMITY ===")
    for i, wp in enumerate(wps):
        closest_dist = 999
        closest_t = 0
        for s in samples:
            d = dist(s['odom']['x'], s['odom']['y'], wp['x'], wp['y'])
            if d < closest_dist:
                closest_dist = d
                closest_t = (s['t'] - t0) / 1000
        reached = "OK" if closest_dist < 0.20 else "MISS"
        print(f"  WP{i}: closest={closest_dist:.3f}m at t={closest_t:.1f}s [{reached}]")
    print()

    # Position range
    xs = [s['odom']['x'] for s in samples]
    ys = [s['odom']['y'] for s in samples]
    print(f"=== RANGE === X:[{min(xs):.3f}, {max(xs):.3f}] Y:[{min(ys):.3f}, {max(ys):.3f}]")
    last = samples[-1]
    print(f"Final: ({last['odom']['x']:.3f}, {last['odom']['y']:.3f}) yaw={math.degrees(last['odom']['yaw']):.1f}")
    print()

    # Phases
    phases = detect_phases(samples, t0)
    print("=== PHASES ===")
    for i, p in enumerate(phases):
        dur = p['t_end'] - p['t_start']
        if dur < 0.3:
            continue  # skip micro-phases
        odom = p.get('odom_end', p['odom_start'])
        print(f"  [{i:2d}] {p['type']:>8} t={p['t_start']:5.1f}-{p['t_end']:5.1f}s ({dur:4.1f}s) "
              f"pos=({odom['x']:.2f}, {odom['y']:.2f}) yaw={math.degrees(odom['yaw']):.1f}")
    print()


def print_trajectory(data, t_start=0, t_end=9999, interval=1.0):
    samples = data['samples']
    ms = data.get('motorSamples', [])
    wps = data.get('waypointsXY', [])
    t0 = samples[0]['t']

    print(f"{'t':>5} {'x':>7} {'y':>7} {'yaw':>6} {'cmd_l':>6} {'cmd_a':>7} {'motL':>6} {'motR':>6} {'d_wp':>6}")

    prev_t = t_start - interval
    for s in samples:
        t_rel = (s['t'] - t0) / 1000
        if t_rel < t_start or t_rel > t_end:
            continue
        if t_rel - prev_t < interval:
            continue
        prev_t = t_rel

        o = s['odom']
        c = s['cmd']
        m = find_closest_motor(ms, s['t'])

        # Distance to closest upcoming waypoint
        min_d = 999
        for wp in wps:
            d = dist(o['x'], o['y'], wp['x'], wp['y'])
            if d < min_d:
                min_d = d

        print(f"{t_rel:5.1f} {o['x']:7.3f} {o['y']:7.3f} {math.degrees(o['yaw']):6.1f} "
              f"{c['l']:6.3f} {c['a']:7.3f} {m['left']:6.1f} {m['right']:6.1f} {min_d:6.2f}")


def print_raw(data, t_start, t_end):
    """Samples cada 0.5s con motores, para ver detalle fino."""
    print_trajectory(data, t_start, t_end, interval=0.5)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Analyze Buey telemetry recording')
    parser.add_argument('--summary', action='store_true', help='Print summary only')
    parser.add_argument('--raw', nargs=2, type=float, metavar=('T_START', 'T_END'),
                        help='Raw samples between t_start and t_end (0.5s interval)')
    parser.add_argument('--trajectory', nargs='?', const='full', metavar='INTERVAL',
                        help='Full trajectory (default 1s interval)')
    parser.add_argument('--interval', type=float, default=1.0,
                        help='Interval in seconds for trajectory output')
    args = parser.parse_args()

    data = fetch_recording()

    if args.raw:
        print_raw(data, args.raw[0], args.raw[1])
    elif args.summary:
        print_summary(data)
    elif args.trajectory is not None:
        print_trajectory(data, interval=args.interval)
    else:
        # Default: summary + trajectory
        print_summary(data)
        print("=== TRAJECTORY (1s) ===")
        print_trajectory(data)
