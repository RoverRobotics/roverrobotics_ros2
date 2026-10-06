import glob
import os
import select
import threading
import time


class PadHeartbeat:
    # a connected pad streams raw HID reports even with idle sticks, so a gap means the link has stalled

    def __init__(self, logger):
        self._logger = logger
        self._last = None
        self._path = None
        self._warned = False
        threading.Thread(target=self._run, daemon=True).start()

    @staticmethod
    def _find():
        # the hidraw node of the same device as the joystick (js*), whatever its number after a reconnect
        for h in sorted(glob.glob('/sys/class/hidraw/hidraw*')):
            if glob.glob(h + '/device/input/*/js*'):
                return '/dev/' + os.path.basename(h)
        return None

    def age(self):
        return None if self._last is None else time.monotonic() - self._last

    def _run(self):
        while True:
            path = self._find()
            if not path:
                time.sleep(0.5)
                continue
            try:
                fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
            except OSError as e:
                if not self._warned:
                    self._logger.warn(f'pad heartbeat off: cannot read {path} ({e.strerror}); '
                                      'stale stick input will not be detected')
                    self._warned = True
                time.sleep(2.0)
                continue
            if path != self._path:
                self._logger.info(f'pad heartbeat: reading {path}')
                self._path = path
            try:
                while True:
                    r, _, _ = select.select([fd], [], [], 0.5)
                    if r:
                        os.read(fd, 128)
                        self._last = time.monotonic()
            except OSError:
                pass  # disconnected: age keeps growing until the pad is back
            finally:
                os.close(fd)
