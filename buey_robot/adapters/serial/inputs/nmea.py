"""Adquisicion de datos GPS via NMEA por puerto serial.

Sin logica ROS2. Abre el puerto, lee lineas NMEA (GGA y RMC), parsea
coordenadas y llama a on_fix con un dict estandarizado.

El loop de lectura corre en un thread separado para no bloquear el
spin() de ROS2 en el driver que instancia este adapter.

Si en el futuro el GPS llega por MQTT o UDP, se agrega otro adapter
con la misma firma de on_fix — el driver drivers/gps_nmea.py no cambia.
"""

import threading
import time
from typing import Callable, Optional

import serial


class SerialNmeaInput:
    """Lectura NMEA desde puerto serial.

    Parametros
    ----------
    port : str
        Nombre del puerto serial (ej. "/dev/ttyACM0").
    baud : int
        Velocidad en baudios (ej. 9600).
    on_fix : Callable[[dict], None]
        Callback invocado con cada fix GPS valido.
        El dict contiene:
          lat          (float, grados decimales, negativo al sur)
          lon          (float, grados decimales, negativo al oeste)
          alt          (float, metros sobre el nivel del mar)
          quality      (int, 0=no fix, 1=GPS, 2=DGPS, 4=RTK Fixed, 5=RTK Float)
          satellites   (int)
          hdop         (float)
          speed_knots  (float o None, velocidad sobre suelo en nudos, de RMC)
          cog          (float o None, curso sobre suelo en grados 0-360, de RMC)
    timeout : float
        Timeout de lectura serial en segundos.
    reconnect_interval : float
        Segundos entre intentos de reconexion si el puerto falla.
    """

    def __init__(
        self,
        port: str,
        baud: int,
        on_fix: Callable[[dict], None],
        timeout: float = 1.0,
        reconnect_interval: float = 2.0,
        logger=None,
    ):
        self._port_name = port
        self._baud = baud
        self._on_fix = on_fix
        self._timeout = timeout
        self._reconnect_interval = reconnect_interval
        self._logger = logger
        self._last_error: Optional[str] = None

        self._serial: Optional[serial.Serial] = None
        self._connected = False
        self._stop_event = threading.Event()

        # Estado GPS acumulado entre sentencias
        self._lat: Optional[float] = None
        self._lon: Optional[float] = None
        self._alt: float = 0.0
        self._quality: int = 0
        self._satellites: int = 0
        self._hdop: float = 99.9
        self._speed_knots: Optional[float] = None
        self._cog: Optional[float] = None

        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    # ------------------------------------------------------------------
    # API publica
    # ------------------------------------------------------------------

    def stop(self):
        """Detiene el thread de lectura y cierra el puerto."""
        self._stop_event.set()
        if self._serial and self._serial.is_open:
            try:
                self._serial.close()
            except Exception:
                pass

    # ------------------------------------------------------------------
    # Thread principal
    # ------------------------------------------------------------------

    def _run_loop(self):
        """Loop que conecta, lee y reconecta en caso de error."""
        while not self._stop_event.is_set():
            if not self._connected:
                self._try_connect()
                if not self._connected:
                    time.sleep(self._reconnect_interval)
                    continue
            try:
                line = self._serial.readline()
                try:
                    sentence = line.decode('ascii', errors='ignore').strip()
                    if sentence.startswith('$'):
                        self._process_sentence(sentence)
                except UnicodeDecodeError:
                    pass
            except serial.SerialException:
                self._connected = False
                if self._serial:
                    try:
                        self._serial.close()
                    except Exception:
                        pass
                    self._serial = None
            except Exception:
                pass

    def _try_connect(self):
        try:
            self._serial = serial.Serial(
                port=self._port_name,
                baudrate=self._baud,
                timeout=self._timeout,
            )
            self._connected = True
            if self._logger is not None:
                self._logger.info(f'Serial NMEA conectado: {self._port_name} @ {self._baud}')
            self._last_error = None
        except serial.SerialException as e:
            self._serial = None
            self._connected = False
            err = str(e)
            # Loguear solo cuando cambia el error (evita spam en reintentos)
            if err != self._last_error and self._logger is not None:
                self._logger.error(f'Serial NMEA no conecta a {self._port_name}: {err}')
                self._last_error = err

    # ------------------------------------------------------------------
    # Parseo NMEA
    # ------------------------------------------------------------------

    def _process_sentence(self, sentence: str):
        if not self._verify_checksum(sentence):
            return
        parts = sentence.split(',')
        if len(parts) < 3:
            return
        sentence_type = parts[0]
        if sentence_type in ('$GPGGA', '$GNGGA'):
            self._process_gga(parts)
        elif sentence_type in ('$GPRMC', '$GNRMC'):
            self._process_rmc(parts)

    def _process_gga(self, parts: list):
        """Sentencia GGA: posicion, calidad, satelites, altitud."""
        if len(parts) < 11:
            return
        try:
            if parts[6]:
                self._quality = int(parts[6])
            if parts[7]:
                self._satellites = int(parts[7])
            if parts[8]:
                self._hdop = float(parts[8])
            if parts[2] and parts[3]:
                lat = self._parse_coordinate(parts[2], parts[3])
                if lat is not None:
                    self._lat = lat
            if parts[4] and parts[5]:
                lon = self._parse_coordinate(parts[4], parts[5])
                if lon is not None:
                    self._lon = lon
            if parts[9]:
                self._alt = float(parts[9])
            self._emit_fix()
        except (ValueError, IndexError):
            pass

    def _process_rmc(self, parts: list):
        """Sentencia RMC: posicion, velocidad (SOG) y curso (COG).

        Campos RMC:
          parts[7] = velocidad sobre suelo en nudos (SOG)
          parts[8] = curso sobre suelo en grados (COG, 0-360 desde Norte)
        """
        if len(parts) < 9:
            return
        try:
            if parts[2] != 'A':
                return
            if parts[3] and parts[4]:
                lat = self._parse_coordinate(parts[3], parts[4])
                if lat is not None:
                    self._lat = lat
            if parts[5] and parts[6]:
                lon = self._parse_coordinate(parts[5], parts[6])
                if lon is not None:
                    self._lon = lon
            if parts[7]:
                self._speed_knots = float(parts[7])
            if parts[8]:
                self._cog = float(parts[8])
            if self._quality > 0:
                self._emit_fix()
        except (ValueError, IndexError):
            pass

    def _emit_fix(self):
        """Llama a on_fix con el estado actual.

        Emite siempre que haya datos GGA (incluso sin fix), pasando lat/lon=None
        cuando todavia no hay coordenadas. El driver decide que hacer con
        cada caso (MQTT vs ROS /gps/fix).
        """
        self._on_fix({
            'lat':        self._lat,
            'lon':        self._lon,
            'alt':        self._alt,
            'quality':    self._quality,
            'satellites': self._satellites,
            'hdop':       self._hdop,
            'speed_knots': self._speed_knots,
            'cog':        self._cog,
        })

    # ------------------------------------------------------------------
    # Utilidades
    # ------------------------------------------------------------------

    @staticmethod
    def _parse_coordinate(coord_str: str, direction: str) -> Optional[float]:
        """Convierte coordenada NMEA ddmm.mmmm a grados decimales."""
        try:
            if direction in ('N', 'S'):
                degrees = float(coord_str[:2])
                minutes = float(coord_str[2:])
            else:
                degrees = float(coord_str[:3])
                minutes = float(coord_str[3:])
            decimal = degrees + (minutes / 60.0)
            if direction in ('S', 'W'):
                decimal = -decimal
            return decimal
        except (ValueError, IndexError):
            return None

    @staticmethod
    def _verify_checksum(sentence: str) -> bool:
        """Verifica checksum NMEA XOR."""
        if '*' not in sentence:
            return False
        try:
            data, checksum = sentence.split('*')
            data = data[1:]
            calc = 0
            for char in data:
                calc ^= ord(char)
            return calc == int(checksum, 16)
        except (ValueError, IndexError):
            return False
