"""Lee lineas de un puerto serial: abre, reconecta solo, y llama on_line(str) por
cada linea. Corre en un thread para no bloquear el spin() de ROS2.
"""

import threading
import time
from typing import Callable, Optional

import serial

from buey_robot.utils.log import TransitionLogger


class SerialLineReader:
    def __init__(
        self,
        port: str,
        baud: int,
        on_line: Callable[[str], None],
        timeout: float = 1.0,
        reconnect_interval: float = 2.0,
        logger=None,
    ):
        self._port_name = port
        self._baud = baud
        self._on_line = on_line
        self._timeout = timeout
        self._reconnect_interval = reconnect_interval
        self._logger = logger
        self._err_log = TransitionLogger(logger)   # dedup de errores repetidos

        self._serial: Optional[serial.Serial] = None
        self._stop_event = threading.Event()

        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        self._close()
        self._thread.join(timeout=self._timeout + 0.5)

    def _run_loop(self):
        while not self._stop_event.is_set():
            if self._serial is None:
                self._try_connect()
                if self._serial is None:
                    time.sleep(self._reconnect_interval)
                continue
            try:
                line = self._serial.readline().decode('ascii', errors='ignore').strip()
                if line:
                    self._on_line(line)
            except serial.SerialException:
                self._close()                              # perdio el puerto -> reconecta
            except Exception as e:
                self._err_log.error(f'Serial: error leyendo/parseando: {e}')

    def _try_connect(self):
        try:
            self._serial = serial.Serial(
                port=self._port_name, baudrate=self._baud, timeout=self._timeout)
            self._err_log.reset()
            if self._logger is not None:
                self._logger.info(f'Serial conectado: {self._port_name} @ {self._baud}')
        except serial.SerialException as e:
            self._serial = None
            self._err_log.error(f'Serial no conecta a {self._port_name}: {e}')

    def _close(self):
        if self._serial:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None
