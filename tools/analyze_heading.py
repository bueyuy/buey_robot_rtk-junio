"""Analisis/prediccion del heading GPS (COG) sobre recordings de telemetria.

Evalua el cambio "heading solo-COG" (use_imu_heading=false) con umbral mas bajo,
usando datos reales del recording (gpsSamples + imuSamples). Es de SOLO LECTURA:
no toca el robot ni la config, solo simula que heading saldria.

Compara, en grados ENU (0=Este, antihorario, convencion de rtk.py):
  - cog_enu       = 90 - gpsSamples.cog      (lo que deriva rtk.py de msg.track)
  - course_pos    = course real de los deltas lat/lon (ground truth, gateado por
                    desplazamiento minimo: a baja velocidad el ruido RTK lo corrompe)
  - headingGps    = imuSamples.headingGps    (lo grabado, ya ENU)
  - headingImu    = imuSamples.headingImu    (lo grabado, ya ENU)

Y reporta cobertura del heading GPS (% de muestras sobre cada umbral de velocidad)
y el sesgo/ruido de cada fuente vs el course real, global y por tramo recto.

Uso:
    python3 tools/analyze_heading.py resource/telemetry_2026-06-10T18-01-24.json
    python3 tools/analyze_heading.py <file> --thresholds 0.12 0.25 0.30
    python3 tools/analyze_heading.py <file> --csv out.csv
"""

import json
import math
import argparse
import statistics

# Desplazamiento minimo (m) entre fixes para confiar en el course de posicion.
# Por debajo de esto el jitter RTK (~1-2cm) domina y el angulo es basura.
MIN_DISP_M = 0.10


def norm180(a):
    """Normaliza un angulo en grados a (-180, 180]."""
    return (a + 180.0) % 360.0 - 180.0


def circ_mean(angs):
    s = sum(math.sin(math.radians(a)) for a in angs)
    c = sum(math.cos(math.radians(a)) for a in angs)
    return math.degrees(math.atan2(s, c))


def circ_std(angs):
    n = len(angs)
    s = sum(math.sin(math.radians(a)) for a in angs) / n
    c = sum(math.cos(math.radians(a)) for a in angs) / n
    r = math.hypot(s, c)
    return math.degrees(math.sqrt(max(0.0, -2.0 * math.log(max(r, 1e-9)))))


def latlon_to_en(gps):
    """Proyeccion equirectangular local: devuelve fn(sample)->(este_m, norte_m)."""
    lat0 = gps[0]['lat']
    m_lat = 111320.0
    m_lon = 111320.0 * math.cos(math.radians(lat0))
    return lambda g: (g['lon'] * m_lon, g['lat'] * m_lat)


def build_gps_rows(gps):
    """Por cada gpsSample: t, cog_enu, course_pos (o None), speed (m/s)."""
    en = latlon_to_en(gps)
    rows = []
    prev = None
    for g in gps:
        e, n = en(g)
        speed = 0.0
        course_pos = None
        if prev is not None:
            de, dn = e - prev[0], n - prev[1]
            dt = (g['t'] - prev[2]) / 1000.0
            disp = math.hypot(de, dn)
            if dt > 0:
                speed = disp / dt
            if disp > MIN_DISP_M:
                course_pos = math.degrees(math.atan2(dn, de))
        rows.append({
            't': g['t'],
            'cog': g['cog'],
            'cog_enu': norm180(90.0 - g['cog']),
            'course_pos': course_pos,
            'speed': speed,
            'quality': g.get('quality'),
        })
        prev = (e, n, g['t'])
    return rows


def simulate_heading(rows, threshold):
    """Simula heading solo-COG con gate de velocidad: si speed<=thr, mantiene el ultimo.

    Devuelve lista paralela a rows con el heading GPS que rtk.py publicaria
    (sin el filtro exponencial; aproximacion del valor crudo gateado).
    """
    out = []
    last = None
    for r in rows:
        if r['speed'] > threshold:
            last = r['cog_enu']
        out.append(last)
    return out


def coverage(rows, threshold):
    n = len(rows)
    live = sum(1 for r in rows if r['speed'] > threshold)
    return 100.0 * live / n if n else 0.0


def bias_std_vs_truth(rows, key):
    """Sesgo/ruido circular de rows[key] (grados ENU) vs course_pos (ground truth).

    Solo usa muestras con course_pos disponible (robot en movimiento real).
    """
    diffs = [norm180(r[key] - r['course_pos'])
             for r in rows if r['course_pos'] is not None and r[key] is not None]
    if not diffs:
        return None
    return circ_mean(diffs), circ_std(diffs), len(diffs)


def nearest_imu(imu, t):
    """imuSample mas cercano por timestamp (lineal; los recordings son chicos)."""
    best = None
    bestd = None
    for s in imu:
        d = abs(s['t'] - t)
        if bestd is None or d < bestd:
            bestd, best = d, s
    return best


def main():
    ap = argparse.ArgumentParser(description='Predice el heading solo-COG sobre un recording')
    ap.add_argument('file', help='Ruta al telemetry_*.json')
    ap.add_argument('--thresholds', nargs='+', type=float, default=[0.12, 0.25, 0.30],
                    help='Umbrales de velocidad a comparar (m/s)')
    ap.add_argument('--new', type=float, default=0.12,
                    help='Umbral nuevo propuesto (m/s) para simular el heading')
    ap.add_argument('--csv', help='Volcar columnas alineadas a un CSV')
    args = ap.parse_args()

    with open(args.file) as f:
        data = json.load(f)

    gps = data.get('gpsSamples') or []
    imu = data.get('imuSamples') or []
    if not gps:
        print('ERROR: el recording no tiene gpsSamples (formato inesperado)')
        return

    note = data.get('metadata', {}).get('testNote', '')
    dur = (gps[-1]['t'] - gps[0]['t']) / 1000.0
    print(f"=== {args.file.split('/')[-1]} ===")
    print(f"Note: {note}")
    print(f"Duracion GPS: {dur:.1f}s | gpsSamples={len(gps)} | imuSamples={len(imu)}")
    rtk_fix = sum(1 for s in imu if s.get('rtkFix'))
    if imu:
        print(f"rtkFix=true: {rtk_fix}/{len(imu)} ({100*rtk_fix/len(imu):.0f}%)")
    print()

    rows = build_gps_rows(gps)

    # Velocidades
    spds = sorted(r['speed'] for r in rows[1:])
    if spds:
        print("=== VELOCIDAD (derivada de posicion GPS) ===")
        print(f"  min={spds[0]:.2f}  mediana={statistics.median(spds):.2f}  "
              f"media={statistics.mean(spds):.2f}  max={spds[-1]:.2f} m/s")
    print()

    # Cobertura por umbral
    print("=== COBERTURA DEL HEADING GPS (% muestras con vel>umbral) ===")
    for thr in args.thresholds:
        print(f"  umbral {thr:.2f} m/s -> {coverage(rows, thr):.0f}% del tiempo con heading vivo")
    print()

    # Sesgo/ruido de cada fuente vs course real de posicion
    print("=== SESGO / RUIDO vs COURSE REAL DE POSICION (en movimiento) ===")
    bs = bias_std_vs_truth(rows, 'cog_enu')
    if bs:
        print(f"  COG (90-cog):        sesgo={bs[0]:+6.1f}  std={bs[1]:5.1f}  n={bs[2]}")

    # headingGps / headingImu grabados: alinear por timestamp al gpsSample
    for r in rows:
        s = nearest_imu(imu, r['t']) if imu else None
        r['headingGps'] = s.get('headingGps') if s else None
        r['headingImu'] = s.get('headingImu') if s else None
    for key, label in [('headingGps', 'headingGps grabado'),
                       ('headingImu', 'headingImu grabado')]:
        bs = bias_std_vs_truth(rows, key)
        if bs:
            print(f"  {label:18}: sesgo={bs[0]:+6.1f}  std={bs[1]:5.1f}  n={bs[2]}")
    print()

    # IMU vs COG (cuanto se aparta el IMU del COG, que es lo que la fusion inyectaba)
    imu_vs_cog = [norm180(r['headingImu'] - r['cog_enu'])
                  for r in rows if r['headingImu'] is not None and r['speed'] > args.new]
    if imu_vs_cog:
        print(f"headingImu - COG (vel>{args.new}): sesgo={circ_mean(imu_vs_cog):+.1f}  "
              f"std={circ_std(imu_vs_cog):.1f}  n={len(imu_vs_cog)}")
        print("  -> esto es el error que la fusion (use_imu_heading=true) metia sobre el COG")
    print()

    # Simulacion del heading nuevo (solo-COG, gate args.new) vs course real
    sim = simulate_heading(rows, args.new)
    diffs = [norm180(h - r['course_pos'])
             for h, r in zip(sim, rows)
             if h is not None and r['course_pos'] is not None]
    if diffs:
        print(f"=== HEADING SIMULADO solo-COG (umbral {args.new}) vs course real ===")
        print(f"  sesgo={circ_mean(diffs):+.1f}  std={circ_std(diffs):.1f}  n={len(diffs)}")
        print(f"  cobertura con umbral {args.new}: {coverage(rows, args.new):.0f}%")
    print()

    if args.csv:
        import csv
        with open(args.csv, 'w', newline='') as fh:
            w = csv.writer(fh)
            w.writerow(['t', 'cog', 'cog_enu', 'course_pos', 'speed',
                        'headingGps', 'headingImu', 'sim_heading', 'quality'])
            for r, h in zip(rows, sim):
                w.writerow([r['t'], r['cog'], f"{r['cog_enu']:.1f}",
                            '' if r['course_pos'] is None else f"{r['course_pos']:.1f}",
                            f"{r['speed']:.3f}",
                            '' if r['headingGps'] is None else f"{r['headingGps']:.1f}",
                            '' if r['headingImu'] is None else f"{r['headingImu']:.1f}",
                            '' if h is None else f"{h:.1f}", r['quality']])
        print(f"CSV escrito en {args.csv}")


if __name__ == '__main__':
    main()
