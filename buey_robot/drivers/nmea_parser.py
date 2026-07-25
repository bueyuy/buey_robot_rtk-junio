"""Parser NMEA (GGA + VTG) -> dict de fix. Acumula estado entre sentencias;
feed(sentence) emite el fix al llegar una VTG con calidad (el receptor no manda
RMC; COG/SOG vienen por VTG), si no None.

fix: lat, lon (grados decimales o None), alt (m), quality (0/1/2/4/5),
satellites, hdop, speed_knots (o None), cog (o None).
"""

from typing import Optional


class NmeaParser:
    def __init__(self):
        self._lat: Optional[float] = None
        self._lon: Optional[float] = None
        self._alt: float = 0.0
        self._quality: int = 0
        self._satellites: int = 0
        self._hdop: float = 99.9
        self._speed_knots: Optional[float] = None
        self._cog: Optional[float] = None

    def feed(self, sentence: str) -> Optional[dict]:
        if not sentence.startswith('$') or not self._verify_checksum(sentence):
            return None
        parts = sentence.split(',')
        if len(parts) < 3:
            return None
        talker = parts[0]                          # $GxGGA / $GxVTG (GP/GN/GL segun receptor)
        if talker.endswith('GGA'):
            self._parse_gga(parts)
        elif talker.endswith('VTG'):
            self._parse_vtg(parts)
            if self._quality > 0:
                return self._fix()
        return None

    def _parse_gga(self, parts: list):
        # GGA: 2=lat 3=N/S 4=lon 5=E/W 6=quality 7=sats 8=hdop 9=alt
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
        except (ValueError, IndexError):
            pass

    def _parse_vtg(self, parts: list):
        # VTG: 1=COG(grados) 5=SOG(nudos). Quieto, el receptor deja COG vacio: se conserva.
        if len(parts) < 6:
            return
        try:
            if parts[1]:
                self._cog = float(parts[1])
            if parts[5]:
                self._speed_knots = float(parts[5])
        except (ValueError, IndexError):
            pass

    def _fix(self) -> dict:
        return {
            'lat': self._lat, 'lon': self._lon, 'alt': self._alt,
            'quality': self._quality, 'satellites': self._satellites,
            'hdop': self._hdop, 'speed_knots': self._speed_knots, 'cog': self._cog,
        }

    @staticmethod
    def _parse_coordinate(coord: str, direction: str) -> Optional[float]:
        try:
            if direction in ('N', 'S'):
                deg, minutes = float(coord[:2]), float(coord[2:])
            else:
                deg, minutes = float(coord[:3]), float(coord[3:])
            decimal = deg + minutes / 60.0
            return -decimal if direction in ('S', 'W') else decimal
        except (ValueError, IndexError):
            return None

    @staticmethod
    def _verify_checksum(sentence: str) -> bool:
        if '*' not in sentence:
            return False
        try:
            data, checksum = sentence.split('*')
            calc = 0
            for char in data[1:]:
                calc ^= ord(char)
            return calc == int(checksum, 16)
        except (ValueError, IndexError):
            return False
