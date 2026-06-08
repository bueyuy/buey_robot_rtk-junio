#!/usr/bin/env python3
"""
Test NMEA Serial: Script de prueba para verificar lectura de GPS por serial.

Este script NO requiere ROS2, solo PySerial. Útil para:
- Verificar que el GPS está conectado correctamente
- Identificar el puerto serial correcto
- Comprobar el baudrate adecuado
- Ver datos NMEA en crudo
"""

import serial
import time
import argparse
import sys


def parse_nmea_coordinate(coord_str, direction):
    """Convierte coordenada NMEA a decimal."""
    try:
        if direction in ['N', 'S']:
            degrees = float(coord_str[:2])
            minutes = float(coord_str[2:])
        else:
            degrees = float(coord_str[:3])
            minutes = float(coord_str[3:])

        decimal = degrees + (minutes / 60.0)
        if direction in ['S', 'W']:
            decimal = -decimal
        return decimal
    except:
        return None


def parse_gga(parts):
    """Parsea sentencia GGA y retorna dict con datos."""
    if len(parts) < 11:
        return None

    try:
        data = {
            'time': parts[1] if parts[1] else 'N/A',
            'lat': parse_nmea_coordinate(parts[2], parts[3]) if parts[2] else None,
            'lon': parse_nmea_coordinate(parts[4], parts[5]) if parts[4] else None,
            'quality': int(parts[6]) if parts[6] else 0,
            'sats': int(parts[7]) if parts[7] else 0,
            'hdop': float(parts[8]) if parts[8] else 99.9,
            'alt': float(parts[9]) if parts[9] else 0.0,
        }
        return data
    except:
        return None


def quality_name(quality):
    """Convierte código de calidad a nombre."""
    names = {
        0: 'No Fix',
        1: 'GPS',
        2: 'DGPS',
        4: 'RTK Fixed',
        5: 'RTK Float',
    }
    return names.get(quality, f'Quality {quality}')


def test_serial_port(port, baudrate, duration=60):
    """
    Prueba lectura del puerto serial.

    Args:
        port: Puerto serial (ej: /dev/ttyUSB0)
        baudrate: Velocidad en baudios (ej: 9600)
        duration: Duración de prueba en segundos
    """
    print(f"\n{'='*70}")
    print(f"Test NMEA Serial GPS")
    print(f"{'='*70}")
    print(f"Puerto:   {port}")
    print(f"Baudrate: {baudrate}")
    print(f"Duración: {duration}s")
    print(f"{'='*70}\n")

    try:
        # Abrir puerto serial
        print(f"Abriendo puerto {port}...")
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"✓ Puerto abierto correctamente\n")

        # Estadísticas
        total_lines = 0
        nmea_lines = 0
        gga_count = 0
        rmc_count = 0
        last_gga = None

        start_time = time.time()

        print(f"{'Tiempo':<8} {'Tipo':<10} {'Lat':<12} {'Lon':<12} {'Alt':<8} {'Calidad':<12} {'Sats':<5}")
        print(f"{'-'*70}")

        # Leer datos
        while (time.time() - start_time) < duration:
            if ser.in_waiting > 0:
                line = ser.readline()
                total_lines += 1

                try:
                    nmea_sentence = line.decode('ascii', errors='ignore').strip()

                    if nmea_sentence.startswith('$'):
                        nmea_lines += 1
                        parts = nmea_sentence.split(',')

                        # Mostrar sentencia completa cada 10 mensajes
                        if nmea_lines % 10 == 1:
                            print(f"\nRaw: {nmea_sentence[:80]}{'...' if len(nmea_sentence) > 80 else ''}")

                        # Parsear GGA
                        if parts[0] in ['$GPGGA', '$GNGGA']:
                            gga_count += 1
                            data = parse_gga(parts)

                            if data:
                                last_gga = data
                                elapsed = time.time() - start_time

                                print(f"{elapsed:>7.1f}s {parts[0]:<10} "
                                      f"{data['lat']:>11.6f} {data['lon']:>11.6f} "
                                      f"{data['alt']:>7.1f} {quality_name(data['quality']):<12} "
                                      f"{data['sats']:>4}")

                        # Contar RMC
                        elif parts[0] in ['$GPRMC', '$GNRMC']:
                            rmc_count += 1

                except UnicodeDecodeError:
                    pass

        # Cerrar puerto
        ser.close()

        # Resumen
        print(f"\n{'='*70}")
        print(f"Resumen de la prueba")
        print(f"{'='*70}")
        print(f"Líneas totales:       {total_lines}")
        print(f"Sentencias NMEA:      {nmea_lines}")
        print(f"  - GGA (posición):   {gga_count}")
        print(f"  - RMC (mínimo):     {rmc_count}")
        print(f"{'='*70}")

        if last_gga:
            print(f"\nÚltima posición GPS:")
            print(f"  Latitud:   {last_gga['lat']:.8f}°")
            print(f"  Longitud:  {last_gga['lon']:.8f}°")
            print(f"  Altitud:   {last_gga['alt']:.2f} m")
            print(f"  Calidad:   {quality_name(last_gga['quality'])}")
            print(f"  Satélites: {last_gga['sats']}")
            print(f"  HDOP:      {last_gga['hdop']:.1f}")

        # Diagnóstico
        print(f"\n{'='*70}")
        print(f"Diagnóstico")
        print(f"{'='*70}")

        if nmea_lines == 0:
            print("✗ No se recibieron sentencias NMEA")
            print("  → Verificar baudrate (probar 4800, 9600, 38400, 115200)")
            print("  → Verificar que el GPS está encendido")
            print("  → Revisar conexión del cable")
            return False

        elif gga_count == 0:
            print("✗ Se recibieron datos NMEA pero sin GGA")
            print("  → GPS podría estar en modo diferente")
            print("  → Verificar configuración del GPS")
            return False

        elif last_gga and last_gga['quality'] == 0:
            print("⚠ GPS sin fix")
            print("  → Mover GPS a zona con visibilidad del cielo")
            print("  → Esperar unos minutos para adquisición de satélites")
            return True

        elif last_gga and last_gga['quality'] in [1, 2]:
            print(f"✓ GPS funcionando con fix {quality_name(last_gga['quality'])}")
            print("  → Precisión: ~2-5 metros")
            if last_gga['quality'] == 1:
                print("  → Para RTK, configurar correcciones NTRIP/base station")
            return True

        elif last_gga and last_gga['quality'] >= 4:
            print(f"✓✓ GPS RTK funcionando perfectamente!")
            print(f"  → Calidad: {quality_name(last_gga['quality'])}")
            print("  → Precisión: 2-20 cm")
            return True

        else:
            print("⚠ Datos recibidos pero estado desconocido")
            return True

    except serial.SerialException as e:
        print(f"\n✗ Error abriendo puerto serial: {e}")
        print(f"\nPosibles soluciones:")
        print(f"  1. Verificar que el dispositivo está conectado:")
        print(f"     Linux:   ls -l /dev/ttyUSB* /dev/ttyACM*")
        print(f"     macOS:   ls /dev/cu.*")
        print(f"     Windows: Administrador de Dispositivos")
        print(f"  2. Verificar permisos (Linux):")
        print(f"     sudo usermod -a -G dialout $USER")
        print(f"     (reiniciar sesión después)")
        print(f"  3. Verificar que ningún otro programa usa el puerto:")
        print(f"     Linux: sudo lsof {port}")
        return False

    except KeyboardInterrupt:
        print("\n\nPrueba interrumpida por usuario")
        if 'ser' in locals() and ser.is_open:
            ser.close()
        return False

    except Exception as e:
        print(f"\n✗ Error inesperado: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    parser = argparse.ArgumentParser(
        description='Test de lectura NMEA por puerto serial',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Ejemplos de uso:

  # Probar puerto por defecto (/dev/ttyUSB0 @ 9600)
  python3 test_nmea_serial.py

  # Especificar puerto y baudrate
  python3 test_nmea_serial.py -p /dev/ttyACM0 -b 38400

  # Prueba más larga (5 minutos)
  python3 test_nmea_serial.py -d 300

  # Listar puertos disponibles
  python3 test_nmea_serial.py --list-ports

Puertos comunes:
  Linux:   /dev/ttyUSB0, /dev/ttyACM0, /dev/serial0
  macOS:   /dev/cu.usbserial-XXXX
  Windows: COM3, COM4, COM5

Baudrates comunes:
  4800, 9600, 19200, 38400, 57600, 115200
  (La mayoría de GPS usan 9600 por defecto)
        """
    )

    parser.add_argument('-p', '--port',
                        default='/dev/ttyUSB0',
                        help='Puerto serial (default: /dev/ttyUSB0)')

    parser.add_argument('-b', '--baudrate',
                        type=int,
                        default=9600,
                        help='Baudrate (default: 9600)')

    parser.add_argument('-d', '--duration',
                        type=int,
                        default=60,
                        help='Duración de prueba en segundos (default: 60)')

    parser.add_argument('--list-ports',
                        action='store_true',
                        help='Listar puertos seriales disponibles')

    args = parser.parse_args()

    # Listar puertos si se solicita
    if args.list_ports:
        try:
            import serial.tools.list_ports
            ports = serial.tools.list_ports.comports()

            print("\nPuertos seriales disponibles:")
            print(f"{'='*70}")

            if ports:
                for port in ports:
                    print(f"Puerto: {port.device}")
                    print(f"  Descripción: {port.description}")
                    print(f"  Hardware ID: {port.hwid}")
                    print()
            else:
                print("No se encontraron puertos seriales")

            print(f"{'='*70}")
        except ImportError:
            print("✗ pyserial no está instalado")
            print("Instalar con: pip3 install pyserial")

        return

    # Ejecutar test
    success = test_serial_port(args.port, args.baudrate, args.duration)

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
