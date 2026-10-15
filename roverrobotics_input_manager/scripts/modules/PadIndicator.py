import time

from .PadFeedback import (PadFeedback, read_pad_battery,
                          TRIGGER_OFF, TRIGGER_RESIST, TRIGGER_WALL)

# Robot state -> light bar colour and rumble. One state is shown at a time, by
# priority, so the light bar stays readable. Colours are paired with a blink
# rate, never carried by hue alone.

FAULT = 'fault'
ESTOP = 'estop'
LINK_STOP = 'link_stop'
TIP_DANGER = 'tip_danger'
TIP_WARN = 'tip_warn'
WAIT_CENTRE = 'wait_centre'
NO_DRIVER = 'no_driver'
AUTONOMY = 'autonomy'
BATT_CRIT = 'batt_crit'
PAD_BATT = 'pad_batt'
BATT_LOW = 'batt_low'
CHARGING = 'charging'
DRIVING = 'driving'
IDLE = 'idle'

PRIORITY = (FAULT, ESTOP, LINK_STOP, TIP_DANGER, TIP_WARN, WAIT_CENTRE, NO_DRIVER,
            AUTONOMY, BATT_CRIT, PAD_BATT, BATT_LOW, CHARGING, DRIVING, IDLE)

# state: (r, g, b), blink Hz
LOOK = {
    # Red: stopped, and the pad can clear it. Magenta: nothing usable behind the
    # pad and no button will help - the same family as the latched STACK_DOWN
    # below, told apart by rhythm. A misread inside a family costs nothing.
    FAULT:       ((255, 0, 255), 4.0),
    ESTOP:       ((255, 0, 0), 0.0),
    LINK_STOP:   ((255, 0, 0), 1.0),
    TIP_DANGER:  ((255, 30, 0), 6.0),
    TIP_WARN:    ((255, 120, 0), 2.0),
    WAIT_CENTRE: ((255, 90, 0), 0.0),     # orange, pulled away from the pad-battery yellow
    NO_DRIVER:   ((60, 0, 60), 0.0),      # dim magenta, solid: waiting for the driver at boot
    AUTONOMY:    ((0, 0, 255), 0.0),
    # the robot's energy is one cyan family, told apart by rate; yellow belongs
    # to the pad's own battery so the two are never confused
    BATT_CRIT:   ((0, 200, 200), 3.0),
    # the pad's own battery: yellow like the robot's, but a flash-flash-pause
    # signature so the two are never mistaken for each other
    # a slow 1 s on / 1 s off pulse: quick flashes were unreadable, and colour
    # alone is easy to ignore, so this is paired with a left-motor buzz below
    PAD_BATT:    ((255, 220, 0), 0.5),    # pure yellow, pulled away from the wait-centre orange
    BATT_LOW:    ((0, 200, 200), 1.0),
    CHARGING:    ((0, 150, 150), 0.4),
    DRIVING:     ((0, 255, 0), 0.0),
    IDLE:        ((0, 50, 0), 0.0),
}

# Rumble vocabulary, one place to tune it. A DS4's small motor alone is hard to
# feel, so every cue drives both motors and lasts long enough for them to spin up.
TAP = (120, 220, 90)        # confirmation: throttle step
WARN = (230, 240, 110)      # something refused or faulted
GAP = (0, 0, 90)
STOP = (255, 200, 400)      # e-stop engaged
NAG = (120, 220, 130)       # low battery, every 10-30 s
LINK = (160, 240, 120)      # Bluetooth range going; longer so a lost frame still registers
PAD_NAG = (220, 0, 700)     # left motor only, long: charge the pad before it drops the link
# the robot's battery beats like a pulse on the heavy motor: lub-dub for low,
# and a second beat close behind when it is critical, so it reads as quickening
HEARTBEAT = [(200, 0, 130), (0, 0, 90), (130, 0, 100)]
HEARTBEAT_CRIT = HEARTBEAT + [(0, 0, 260)] + HEARTBEAT

# one-shot rumble on entering a state: [(strong, weak, ms), ...]
ON_ENTRY = {
    FAULT:     [WARN, GAP, WARN, GAP, WARN],
    ESTOP:     [STOP],
    LINK_STOP: [STOP],   # lands only if a write still gets through as the link dies
}

# The magenta family, by rhythm: dim solid = waiting at boot, 4 Hz = live fault
# (driver dead or a VESC mute), 1.25 Hz latched = our software is gone.
STACK_DOWN = (255, 0, 255)
STACK_DOWN_BLINK = (40, 40)      # hardware blink, 400 ms on / 400 ms off = 1.25 Hz

PAD_BATT_LOW_PCT = 20        # the pad reports in coarse steps, so keep the bands wide
PAD_BATT_PULSE_S = 60.0      # one burst a minute: often enough to act on, rare enough to ignore
PAD_BURST_S = 10.0           # five 1 s flashes, then back to the normal state

BATT_LOW_SOC = 0.25
BATT_CRIT_SOC = 0.10
BATT_PULSE_LOW_S = 30.0
BATT_PULSE_CRIT_S = 10.0
# The driver FATALs ~1 s after CAN frames stop (CommCan TIMEOUT_MS_ 1000) and its
# death takes joy_manager down with it, so the cue must fire inside that window.
# /robot_status runs at 60 Hz, so 0.7 s is 40+ missed messages: not a hiccup.
DRIVER_STALE_S = 0.7
TIP_WARN_MARGIN = 0.5
TIP_DANGER_MARGIN = 0.25


class PadIndicator:
    def __init__(self, logger, enabled=True, triggers=True):
        self._log = logger
        self._pad = PadFeedback(logger, enabled)
        self._use_triggers = triggers
        self._state = None
        self._estop = False
        self._wait_centre = False
        self._link_stop = False
        self._pad_stale = False
        self._autonomy = False
        self._moving = False
        self._soc = None
        self._charging = False
        self._driver_seen = None
        self._hardware_fault = False
        self._tip_margin = None
        self._next_batt_pulse = 0.0
        self._pad_pct = None
        self._pad_charging = False
        self._next_pad_pulse = 0.0
        self._pad_burst_until = 0.0
        self._trigger_mode = None

    # ---- inputs ----

    def set_estop(self, engaged):
        self._estop = bool(engaged)

    def set_wait_centre(self, waiting):
        self._wait_centre = bool(waiting)

    def set_link_stop(self, stopped):
        # the pad link dropped and the stop has not been re-armed yet
        self._link_stop = bool(stopped)

    def set_moving(self, moving):
        self._moving = bool(moving)

    def set_autonomy(self, active):
        self._autonomy = bool(active)

    def set_battery(self, soc, charging):
        self._soc = soc
        self._charging = bool(charging)

    def set_hardware_fault(self, faulted):
        # the robot is alive but refuses to move: a stale VESC holds all motors stopped
        self._hardware_fault = bool(faulted)

    def poll_pad_battery(self):
        # cheap sysfs read; the pad reports slowly so this is called about twice a minute
        reading = read_pad_battery()
        if reading is None:
            return None
        self._pad_pct, status = reading
        self._pad_charging = status in ('Charging', 'Full')
        return self._pad_pct

    def driver_alive(self):
        self._driver_seen = time.monotonic()

    def set_tip_margin(self, margin):
        # 1.0 = upright, 0.0 = going over; None disables. DD fork only for now.
        self._tip_margin = margin

    def set_pad_stale(self, stale):
        if stale == self._pad_stale:
            return
        self._pad_stale = stale
        if not stale:
            # the link-loss stop fired while the pad was out of reach; say so now it is back
            self._pad.pattern([(255, 200, 300), (0, 0, 150), (255, 200, 300)])

    # ---- one-shot feedback ----

    def tick_throttle(self):
        self._pad.pattern([TAP])

    def tick_ceiling(self):
        self._pad.pattern([WARN, GAP, WARN])

    def tick_link_warn(self):
        self._pad.pattern([LINK])

    # ---- arbitration ----

    def _wanted(self, now):
        if self._driver_seen is None or now - self._driver_seen > DRIVER_STALE_S:
            return NO_DRIVER if self._driver_seen is None else FAULT
        if self._hardware_fault:
            return FAULT
        if self._estop:
            return ESTOP
        if self._link_stop:
            return LINK_STOP
        if self._tip_margin is not None:
            if self._tip_margin <= TIP_DANGER_MARGIN:
                return TIP_DANGER
            if self._tip_margin <= TIP_WARN_MARGIN:
                return TIP_WARN
        if self._wait_centre:
            return WAIT_CENTRE
        if self._autonomy:
            return AUTONOMY
        if self._charging:
            return CHARGING
        if self._soc is not None and self._soc <= BATT_CRIT_SOC:
            return BATT_CRIT
        if now < self._pad_burst_until:
            return PAD_BATT
        if self._soc is not None and self._soc <= BATT_LOW_SOC:
            return BATT_LOW
        return DRIVING if self._moving else IDLE

    def update(self):
        now = time.monotonic()
        state = self._wanted(now)
        if state != self._state:
            self._state = state
            for step in ON_ENTRY.get(state, []):
                self._pad.pattern([step])
            self._log.debug(f'pad indicator: {state}')
        rgb, blink = LOOK[state]
        self._pad.set_colour(rgb, blink)

        # sustained rumble only for tip-over proximity, scaled by how close it is
        if state in (TIP_WARN, TIP_DANGER) and self._tip_margin is not None:
            closeness = max(0.0, min(1.0, 1.0 - self._tip_margin / TIP_WARN_MARGIN))
            self._pad.set_rumble_level(int(90 + 165 * closeness))   # under ~90 an ERM will not spin
        else:
            self._pad.set_rumble_level(0)

        # battery nags, independent of which state owns the light bar
        if self._soc is not None and not self._charging and self._soc <= BATT_LOW_SOC:
            gap = BATT_PULSE_CRIT_S if self._soc <= BATT_CRIT_SOC else BATT_PULSE_LOW_S
            if now >= self._next_batt_pulse:
                self._next_batt_pulse = now + gap
                self._pad.pattern(HEARTBEAT_CRIT if self._soc <= BATT_CRIT_SOC
                                  else HEARTBEAT)
        else:
            self._next_batt_pulse = 0.0

        # a flat pad drops the link and stops the robot, so nag for it too
        if (self._pad_pct is not None and not self._pad_charging
                and self._pad_pct <= PAD_BATT_LOW_PCT):
            if now >= self._next_pad_pulse:
                self._next_pad_pulse = now + PAD_BATT_PULSE_S
                self._pad_burst_until = now + PAD_BURST_S
                self._log.info(f'pad battery {self._pad_pct}%: warning burst')
                # one buzz per flash, so the whole 10 s burst is felt as well as seen
                burst = []
                for _ in range(5):
                    burst += [PAD_NAG, (0, 0, 1300)]
                self._pad.pattern(burst)
        else:
            self._next_pad_pulse = 0.0
            self._pad_burst_until = 0.0

        self._apply_triggers(state)

    def _apply_triggers(self, state):
        # DualSense only: a trigger you cannot push through when the robot must not move
        if not self._use_triggers or self._pad.model() != 'ds5':
            return
        # params verified by feel on a DualSense 2026-10-15: a wall needs a real
        # span to be noticeable, and start 0 end 2 was imperceptible
        if state in (ESTOP, FAULT, TIP_DANGER):
            mode, params = TRIGGER_WALL, (2, 7, 8)      # hard stop across most of the travel
        elif state == TIP_WARN:
            mode, params = TRIGGER_RESIST, (4, 8)       # resistance from mid-travel
        else:
            mode, params = TRIGGER_OFF, ()
        if mode != self._trigger_mode:
            self._trigger_mode = mode
            self._pad.set_triggers(mode, params)

    def close(self):
        # magenta is used by no other state, so a blinking magenta pad means exactly
        # one thing: the control stack is gone. Blank would read as a pad that is off.
        self._pad.set_rumble_level(0)
        if not self._pad.latch(STACK_DOWN, STACK_DOWN_BLINK):
            self._pad.set_colour(STACK_DOWN)
            time.sleep(0.15)
        self._pad.close()
