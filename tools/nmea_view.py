#!/usr/bin/env python3
"""Visor en vivo de tramas NMEA por serial.

Uso:
    python3 nmea_view.py [puerto] [baud]
    python3 nmea_view.py /dev/ttyACM0 9600   (por defecto)

Muestra las ultimas tramas crudas arriba y un panel parseado (GGA/RMC)
abajo que se refresca en su lugar. Ctrl+C para salir.

OJO: el GPS lo lee un solo proceso a la vez. Si el stack ROS2 esta corriendo
(gps_nmea_driver), detenelo antes o vas a ver tramas partidas.
"""
import sys
import time
from collections import deque

try:
    import serial
except ImportError:
    sys.exit("Falta pyserial: pip install pyserial (o usar el venv del ws)")

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 9600

QUALITY = {0: "No Fix", 1: "GPS", 2: "DGPS", 4: "RTK Fixed", 5: "RTK Float"}

# Verde si bueno, rojo si malo
def c(txt, color):
    codes = {"g": 32, "y": 33, "r": 31, "c": 36, "b": 34, "dim": 90}
    return f"\033[{codes[color]}m{txt}\033[0m"


def parse_coord(s, d):
    try:
        if d in ("N", "S"):
            deg, mins = float(s[:2]), float(s[2:])
        else:
            deg, mins = float(s[:3]), float(s[3:])
        v = deg + mins / 60.0
        return -v if d in ("S", "W") else v
    except (ValueError, IndexError):
        return None


state = {
    "lat": None, "lon": None, "alt": None, "quality": 0, "sats": 0,
    "hdop": None, "sog_kn": None, "cog": None, "rmc_status": "-",
    "n_gga": 0, "n_rmc": 0, "n_other": 0, "n_bad": 0,
}


def checksum_ok(sentence):
    if "*" not in sentence:
        return False
    try:
        data, cks = sentence.split("*")
        calc = 0
        for ch in data[1:]:
            calc ^= ord(ch)
        return calc == int(cks, 16)
    except (ValueError, IndexError):
        return False


def update(sentence):
    p = sentence.split(",")
    t = p[0]
    if t in ("$GPGGA", "$GNGGA") and len(p) >= 11:
        state["n_gga"] += 1
        try:
            state["quality"] = int(p[6]) if p[6] else state["quality"]
            state["sats"] = int(p[7]) if p[7] else state["sats"]
            state["hdop"] = float(p[8]) if p[8] else state["hdop"]
            if p[2] and p[3]:
                state["lat"] = parse_coord(p[2], p[3])
            if p[4] and p[5]:
                state["lon"] = parse_coord(p[4], p[5])
            state["alt"] = float(p[9]) if p[9] else state["alt"]
        except (ValueError, IndexError):
            pass
    elif t in ("$GPRMC", "$GNRMC") and len(p) >= 9:
        state["n_rmc"] += 1
        state["rmc_status"] = p[2] or "-"
        try:
            state["sog_kn"] = float(p[7]) if p[7] else None
            state["cog"] = float(p[8]) if p[8] else None
        except (ValueError, IndexError):
            pass
    else:
        state["n_other"] += 1


def render(raw_lines):
    out = ["\033[2J\033[H"]  # clear + home
    out.append(c(f"  NMEA viewer  {PORT} @ {BAUD}", "c") +
               c("   (Ctrl+C para salir)\n", "dim"))
    out.append(c("  ── Tramas crudas " + "─" * 50, "dim"))
    for ln in raw_lines:
        col = "g" if checksum_ok(ln) else "r"
        out.append("  " + c(ln, col))
    for _ in range(len(raw_lines), raw_lines.maxlen):
        out.append("")
    out.append(c("  ── Parseado " + "─" * 55, "dim"))

    q = state["quality"]
    qname = QUALITY.get(q, f"Q{q}")
    qcol = "g" if q >= 4 else ("y" if q >= 1 else "r")
    sats_col = "g" if state["sats"] >= 6 else "y"
    rmc_col = "g" if state["rmc_status"] == "A" else "r"

    lat = f"{state['lat']:.7f}" if state["lat"] is not None else "—"
    lon = f"{state['lon']:.7f}" if state["lon"] is not None else "—"
    alt = f"{state['alt']:.2f} m" if state["alt"] is not None else "—"
    hdop = f"{state['hdop']:.2f}" if state["hdop"] is not None else "—"
    cog = c(f"{state['cog']:.1f}°", "g") if state["cog"] is not None else c("— (quieto?)", "y")
    sog = (c(f"{state['sog_kn']:.2f} kn ({state['sog_kn']*0.514444:.2f} m/s)", "g")
           if state["sog_kn"] is not None else c("— (quieto?)", "y"))

    out.append(f"  Calidad : {c(qname, qcol)}    Sats: {c(state['sats'], sats_col)}    HDOP: {hdop}")
    out.append(f"  Lat/Lon : {lat}, {lon}")
    out.append(f"  Altitud : {alt}")
    out.append(f"  RMC stat: {c(state['rmc_status'], rmc_col)}  (A=valido, V=invalido)")
    out.append(f"  COG     : {cog}")
    out.append(f"  SOG     : {sog}")
    out.append(c(f"  Conteo  : GGA={state['n_gga']}  RMC={state['n_rmc']}  "
                 f"otras={state['n_other']}  cks_malo={state['n_bad']}", "dim"))
    sys.stdout.write("\n".join(out) + "\n")
    sys.stdout.flush()


def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1.0)
    except serial.SerialException as e:
        sys.exit(c(f"No se pudo abrir {PORT}: {e}\n", "r") +
                 "¿El gps_nmea_driver esta corriendo y tiene el puerto tomado?")
    raw = deque(maxlen=12)
    last = 0.0
    print(c(f"Conectado a {PORT} @ {BAUD}. Esperando tramas...", "g"))
    try:
        while True:
            line = ser.readline().decode("ascii", errors="ignore").strip()
            if line.startswith("$"):
                raw.append(line)
                if not checksum_ok(line):
                    state["n_bad"] += 1
                update(line)
                now = time.time()
                if now - last > 0.2:  # refrescar UI ~5 Hz
                    render(raw)
                    last = now
    except KeyboardInterrupt:
        print(c("\n\nSaliendo.", "dim"))
    finally:
        ser.close()


if __name__ == "__main__":
    main()
