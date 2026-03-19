import micropython
from pyb import Pin
from task_share import Share

S0_INIT = micropython.const(0)
S1_RUN  = micropython.const(1)


class task_bump:
    # 0 = not pressed, 1 = pressed
    # COM (1) = PB12 (pin that switches between other two pins)
    # NO (3) = GND (not pressed = circuit open, pressed = circuit closed)

    def __init__(self, bump_share: Share, pin=Pin.cpu.B12, active_low=True, debounce_count=3):
        self._state = S0_INIT
        self._bump_share = bump_share

        self._active_low = active_low
        self._debounce_count = debounce_count

        if active_low:
            self._pin = Pin(pin, mode=Pin.IN, pull=Pin.PULL_UP)
        else:
            self._pin = Pin(pin, mode=Pin.IN, pull=Pin.PULL_DOWN)

        self._candidate = 0
        self._stable = 0
        self._count = 0

    def raw_pressed(self):
        raw = self._pin.value()
        if self._active_low:
            if raw == 0:
                return 1
            else:
                return 0
        else:
            if raw != 0:
                return 1
            else:
                return 0

    def run(self):
        while True:
            if self._state == S0_INIT:
                self._stable = self.raw_pressed()
                self._candidate = self._stable
                self._count = 0
                self._bump_share.put(self._stable)
                self._state = S1_RUN

            elif self._state == S1_RUN:
                sample = self.raw_pressed()

                if sample == self._candidate:
                    self._count += 1
                else:
                    self._candidate = sample
                    self._count = 1

                if self._count >= self._debounce_count and self._stable != self._candidate:
                    self._stable = self._candidate
                    self._bump_share.put(self._stable)

            yield self._state

