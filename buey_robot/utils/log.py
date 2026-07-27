"""Logger que emite solo cuando cambia el estado (evita spam en loops a N Hz).
Sin key, la clave es el propio mensaje; con key, la transicion la marca la key
(util cuando el texto varia dentro del mismo estado, ej un valor medido)."""

_UNSET = object()


class TransitionLogger:
    def __init__(self, logger):
        self._logger = logger
        self._last = _UNSET

    def info(self, msg, key=None):
        self._emit(msg, key, 'info')

    def warn(self, msg, key=None):
        self._emit(msg, key, 'warn')

    def error(self, msg, key=None):
        self._emit(msg, key, 'error')

    def reset(self):
        self._last = _UNSET

    def _emit(self, msg, key, level):
        k = msg if key is None else key
        if k != self._last:
            if self._logger is not None:
                getattr(self._logger, level)(msg)
            self._last = k
