import fcntl
import glob
import os
import struct
import threading
import time
import zlib

# Light bar, rumble and DualSense trigger effects.
#
# Two backends. A PS4/PS5 pad gets raw HID output reports on its own hidraw node,
# which is the only way to reach the light bar and the triggers without root.
# Anything else the kernel calls a joystick (Xbox, 8BitDo, Logitech) gets rumble
# through the evdev force-feedback interface; those pads have no programmable
# light, so colour is a no-op there.
#
# Both need the pad writable by plugdev (55-roverrobotics.rules). Nothing in this
# file may block or raise into a ROS callback: the same process runs the
# pad-link-loss stop.

DS4 = 'ds4'
DS5 = 'ds5'
GENERIC = 'generic'
_DS4_PRODUCTS = (0x05c4, 0x09cc, 0x0ba0)          # v1, v2, wireless adapter
_DS5_PRODUCTS = (0x0ce6, 0x0df2)                  # DualSense, DualSense Edge
_DS5_NAMES = ('dualsense',)
_DS4_NAMES = ('dualshock', 'wireless controller', 'playstation', 'ps4', 'ps5')

# DS4 valid_flag0
_DS4_F0_MOTOR = 0x01
_DS4_F0_LED = 0x02
_DS4_F0_LED_BLINK = 0x04     # hardware blink: keeps blinking with no host writes
# DS5 valid_flag0 / valid_flag1 / valid_flag2, from hid-playstation.c
_DS5_F0_VIBRATION = 0x01
_DS5_F0_HAPTICS_SELECT = 0x02    # picks classic rumble over the audio-haptics path
_DS5_F0_RIGHT_TRIGGER = 0x04
_DS5_F0_LEFT_TRIGGER = 0x08
_DS5_F1_LIGHTBAR = 0x04
_DS5_F1_PLAYER_LEDS = 0x10
_DS5_F2_VIBRATION2 = 0x04        # newer firmware ignores the v1 bit

def blink_on(blink, now):
    # blink is a rate in Hz, or a repeating on/off pattern in seconds for a
    # signature a steady rate cannot give, e.g. flash-flash-pause
    if isinstance(blink, (tuple, list)):
        total = sum(blink)
        if total <= 0:
            return True
        t = now % total
        lit = True
        for seg in blink:
            if t < seg:
                return lit
            t -= seg
            lit = not lit
        return lit
    return not (blink > 0 and int(now * 2 * blink) % 2)


def read_pad_battery():
    # hid-playstation registers a power supply for the pad; hid-generic does not,
    # so this returns None on a kernel without the PlayStation driver
    for d in glob.glob('/sys/class/power_supply/ps-controller-battery-*'):
        try:
            with open(d + '/capacity') as f:
                pct = int(f.read().strip())
            with open(d + '/status') as f:
                status = f.read().strip()
            return pct, status
        except (OSError, ValueError):
            return None
    return None


TRIGGER_OFF = 0
TRIGGER_RESIST = 1   # params: start 0-9, strength 0-8
TRIGGER_WALL = 2     # params: start 0-9, end 0-9, strength 0-8

_EVIOCSFF = 0x40304580   # _IOW('E', 0x80, struct ff_effect), 48 bytes
_EV_FF = 0x15
_FF_RUMBLE = 0x50


class _FfRumble:
    # evdev force feedback: one reusable effect, magnitudes updated in place

    def __init__(self, path):
        self._fd = os.open(path, os.O_RDWR)
        self._id = -1
        self._playing = False
        try:
            self._upload(0, 0)          # probe: no FF support raises here
        except OSError:
            os.close(self._fd)
            raise

    def _upload(self, strong, weak):
        effect = bytearray(48)
        struct.pack_into('<HhHHHHH', effect, 0, _FF_RUMBLE, self._id, 0, 0, 0, 0, 0)
        struct.pack_into('<HH', effect, 16, strong * 257, weak * 257)
        fcntl.ioctl(self._fd, _EVIOCSFF, effect)
        self._id = struct.unpack_from('<h', effect, 2)[0]

    def set(self, strong, weak):
        if not strong and not weak:
            if self._playing:
                self._play(0)
                self._playing = False
            return
        self._upload(strong, weak)
        self._play(1)
        self._playing = True

    def _play(self, value):
        os.write(self._fd, struct.pack('<qqHHi', 0, 0, _EV_FF, self._id, value))

    def close(self):
        try:
            self.set(0, 0)
        finally:
            os.close(self._fd)


class PadFeedback:
    RATE = 20.0          # writer wake-ups per second
    KEEPALIVE_S = 1.0    # rewrite an unchanged report this often; the driver can revert it

    def __init__(self, logger, enabled=True):
        self._logger = logger
        self._lock = threading.Lock()
        self._colour = (0, 0, 0)
        self._blink_hz = 0.0
        self._level = (0, 0)        # sustained rumble (strong, weak)
        self._queue = []            # one-shot steps: (strong, weak, seconds)
        self._step_until = 0.0
        self._step = None
        self._triggers = (TRIGGER_OFF, ())
        self._player_leds = 0
        self._model = None
        self._path = None
        self._sysfs = None
        self._bt = False
        self._fd = None
        self._ff = None
        self._seq = 0
        self._warned = False
        self._last_write = 0.0
        self._last_buf = None
        self._last_level = None
        self._bad = set()
        self._no_ff = set()
        self._stop = False
        if enabled:
            threading.Thread(target=self._run, daemon=True).start()

    # ---- public API, all non-blocking ----

    def set_colour(self, rgb, blink_hz=0.0):
        with self._lock:
            self._colour = tuple(int(max(0, min(255, c))) for c in rgb)
            self._blink_hz = blink_hz if isinstance(blink_hz, (tuple, list)) else float(blink_hz)

    def set_rumble_level(self, strong, weak=None):
        # sustained rumble, e.g. proportional to a tip-over margin
        weak = strong if weak is None else weak
        with self._lock:
            self._level = (int(max(0, min(255, strong))), int(max(0, min(255, weak))))

    def pulse(self, strong, weak, ms):
        self.pattern([(strong, weak, ms)])

    def pattern(self, steps):
        # steps = [(strong, weak, ms), ...]; a (0, 0, ms) step is a gap
        with self._lock:
            if len(self._queue) > 12:
                return
            for strong, weak, ms in steps:
                self._queue.append((int(strong), int(weak), ms / 1000.0))

    def set_triggers(self, mode, params=()):
        with self._lock:
            self._triggers = (mode, tuple(params))

    def set_player_leds(self, bits):
        with self._lock:
            self._player_leds = int(bits) & 0x1f

    def model(self):
        return self._model

    def has_light(self):
        return self._model in (DS4, DS5)

    def latch(self, rgb, blink=None):
        # Final state, written synchronously as this process goes away. The light bar
        # holds whatever was written last, so leaving it green would read as healthy.
        # On a DS4 the hardware keeps blinking with no host writes; a DualSense cannot,
        # so it gets a solid colour.
        self._stop = True
        time.sleep(0.08)                 # let the writer thread fall out of its loop
        if self._fd is None or self._model is None:
            return False
        try:
            rgb = tuple(int(max(0, min(255, c))) for c in rgb)
            buf = self._build(rgb, 0, 0, (TRIGGER_OFF, ()), 0, blink)
            os.write(self._fd, bytes(buf))
            return True
        except OSError:
            return False

    def close(self):
        self._stop = True

    # ---- device discovery ----

    @staticmethod
    def _identify(hidraw_sysfs):
        try:
            with open(hidraw_sysfs + '/device/uevent') as f:
                fields = dict(line.strip().split('=', 1) for line in f if '=' in line)
        except OSError:
            return None, False
        parts = fields.get('HID_ID', '').split(':')
        if len(parts) != 3:
            return None, False
        bus, vendor, product = (int(p, 16) for p in parts)
        bt = bus == 0x05
        if product in _DS4_PRODUCTS:
            return DS4, bt
        if product in _DS5_PRODUCTS:
            return DS5, bt
        # a clone, or a model newer than this list: go by name and let the first
        # write decide. A pad that rejects the report falls back to rumble.
        name = fields.get('HID_NAME', '').lower()
        if any(n in name for n in _DS5_NAMES):
            return DS5, bt
        if vendor == 0x054c or any(n in name for n in _DS4_NAMES):
            return DS4, bt
        return None, bt

    def _find_playstation(self):
        # the hidraw node of the same device as the joystick, whatever its number after a reconnect
        # (self._bad holds nodes whose first write was rejected)
        for h in sorted(glob.glob('/sys/class/hidraw/hidraw*')):
            if not glob.glob(h + '/device/input/*/js*') or h in self._bad:
                continue
            model, bt = self._identify(h)
            if model:
                return '/dev/' + os.path.basename(h), model, bt, h
        return None, None, False, None

    def _find_joystick_event(self):
        # any pad the kernel exposes as a joystick, PlayStation or not
        for inp in sorted(glob.glob('/sys/class/input/input*')):
            if not glob.glob(inp + '/js*'):
                continue
            for event in sorted(glob.glob(inp + '/event*')):
                path = '/dev/input/' + os.path.basename(event)
                if path not in self._no_ff:
                    return path
        return None

    # ---- report building ----

    def _common_ds4(self, rgb, strong, weak, blink=None):
        c = bytearray(11)
        # the blink flag must be asserted on EVERY report: the pad keeps its last blink
        # setting otherwise, so a later solid colour would inherit the old blink rate
        c[0] = _DS4_F0_MOTOR | _DS4_F0_LED | _DS4_F0_LED_BLINK
        c[3] = weak
        c[4] = strong
        c[5], c[6], c[7] = rgb
        c[8], c[9] = blink if blink else (0, 0)   # on / off, units of 10 ms; 0/0 = steady
        return c

    def _common_ds5(self, rgb, strong, weak, triggers, player_leds):
        c = bytearray(47)
        c[0] = _DS5_F0_VIBRATION | _DS5_F0_HAPTICS_SELECT
        c[1] = _DS5_F1_LIGHTBAR | _DS5_F1_PLAYER_LEDS
        c[38] = _DS5_F2_VIBRATION2
        c[2] = weak
        c[3] = strong
        mode, params = triggers
        if mode != TRIGGER_OFF:
            c[0] |= _DS5_F0_RIGHT_TRIGGER | _DS5_F0_LEFT_TRIGGER
            for base in (10, 21):
                c[base] = mode
                for i, p in enumerate(params[:10]):
                    c[base + 1 + i] = int(p) & 0xff
        # no lightbar_setup here: 0x02 means fade the light OUT, and the driver
        # already clears the Bluetooth startup animation when the pad connects
        c[43] = player_leds
        c[44], c[45], c[46] = rgb
        return c

    def _build(self, rgb, strong, weak, triggers, player_leds, blink=None):
        if self._model == DS4:
            common = self._common_ds4(rgb, strong, weak, blink)
            if not self._bt:
                buf = bytearray(32)
                buf[0] = 0x05
                buf[1:1 + len(common)] = common
                return buf
            buf = bytearray(78)
            buf[0] = 0x11
            buf[1] = 0xc0            # hw_control: HID | CRC32
            buf[3:3 + len(common)] = common
        else:
            common = self._common_ds5(rgb, strong, weak, triggers, player_leds)
            if not self._bt:
                buf = bytearray(48)
                buf[0] = 0x02
                buf[1:1 + len(common)] = common
                return buf
            buf = bytearray(78)
            buf[0] = 0x31
            buf[1] = (self._seq & 0x0f) << 4
            buf[2] = 0x10            # DS_OUTPUT_TAG
            self._seq += 1
            buf[3:3 + len(common)] = common
        crc = zlib.crc32(b'\xa2' + bytes(buf[:74])) & 0xffffffff
        buf[74:78] = crc.to_bytes(4, 'little')
        return buf

    # ---- writer thread ----

    def _wanted(self, now):
        with self._lock:
            rgb, blink, level, triggers, leds = (self._colour, self._blink_hz,
                                                 self._level, self._triggers, self._player_leds)
            if self._step is None or now >= self._step_until:
                self._step = self._queue.pop(0) if self._queue else None
                self._step_until = now + (self._step[2] if self._step else 0.0)
            step = self._step
        if not blink_on(blink, now):
            rgb = (0, 0, 0)
        strong, weak = (step[0], step[1]) if step else level
        return rgb, strong, weak, triggers, leds

    def _open(self):
        path, model, bt, sysfs = self._find_playstation()
        if path:
            try:
                self._fd = os.open(path, os.O_WRONLY | os.O_NONBLOCK)
            except OSError as e:
                self._warn(f'cannot write {path} ({e.strerror}); trying force feedback instead')
                path = None
        if path:
            self._path, self._model, self._bt, self._sysfs = path, model, bt, sysfs
            self._last_buf = None
            self._logger.info(f'pad feedback: {model} on {path} '
                              f'({"bluetooth" if bt else "usb"}), light bar and rumble')
            return True
        event = self._find_joystick_event()
        if not event:
            return False
        try:
            self._ff = _FfRumble(event)
        except OSError as e:
            self._no_ff.add(event)
            self._warn(f'pad feedback off: {event} has no light bar and no rumble '
                       f'({e.strerror}); this pad or kernel cannot do either')
            return False
        self._path, self._model, self._bt = event, model or GENERIC, bt
        self._last_level = None
        self._logger.info(f'pad feedback: {self._model} on {event}, rumble only '
                          '(no programmable light on this pad)')
        return True

    def _warn(self, message):
        if not self._warned:
            self._logger.warn(message)
            self._warned = True

    def _drop(self):
        for closer in (lambda: os.close(self._fd) if self._fd is not None else None,
                       lambda: self._ff.close() if self._ff else None):
            try:
                closer()
            except OSError:
                pass
        self._fd, self._ff, self._path, self._model, self._sysfs = None, None, None, None, None
        self._last_buf, self._last_level = None, None
        with self._lock:
            self._queue.clear()      # a cue for a link that is gone must not fire on reconnect
            self._step = None

    def _run(self):
        period = 1.0 / self.RATE
        while not self._stop:
            if self._path is None:
                if not self._open():
                    time.sleep(1.0)
                    continue
            now = time.monotonic()
            rgb, strong, weak, triggers, leds = self._wanted(now)
            try:
                if self._ff:
                    if (strong, weak) != self._last_level:
                        self._ff.set(strong, weak)
                        self._last_level = (strong, weak)
                else:
                    buf = bytes(self._build(rgb, strong, weak, triggers, leds))
                    if buf != self._last_buf or now - self._last_write > self.KEEPALIVE_S:
                        os.write(self._fd, buf)
                        self._last_write, self._last_buf = now, buf
            except OSError:
                if self._last_buf is None and self._sysfs:
                    # it never accepted a single report: not the pad we took it for
                    self._bad.add(self._sysfs)
                    self._logger.warn(f'pad feedback: {self._path} rejected the '
                                      f'{self._model} report; falling back to rumble only')
                self._drop()      # unplugged or the link dropped: rediscover
                continue
            time.sleep(period)
