from pyb import Switch
import micropython

S0_IDLE = micropython.const(0)
S1_RUN  = micropython.const(1)

class task_start_button:
    def __init__(self, leftMotorGo, rightMotorGo, lf_enable=None, lf_v_share=None,
                 effL=None, effR=None, spL=None, spR=None, start_speed=150.0):
        self._leftMotorGo = leftMotorGo
        self._rightMotorGo = rightMotorGo
        self._lf_enable = lf_enable
        self._lf_v = lf_v_share
        self._effL = effL
        self._effR = effR
        self._spL = spL
        self._spR = spR
        self._start_speed = start_speed

        self._button = Switch()
        self._prev = False
        self._state = S0_IDLE

    def run(self):
        while True:
            pressed = self._button()

            # rising edge detect
            if pressed and not self._prev:
                if self._state == S0_IDLE:
                    self._leftMotorGo.put(1)
                    self._rightMotorGo.put(1)
                    if self._lf_v is not None:
                        self._lf_v.put(self._start_speed)
                    if self._lf_enable is not None:
                        self._lf_enable.put(1)
                    self._state = S1_RUN

                else:
                    if self._lf_enable is not None:
                        self._lf_enable.put(0)
                    self._leftMotorGo.put(0)
                    self._rightMotorGo.put(0)
                    if self._spL is not None:
                        self._spL.put(0)
                    if self._spR is not None:
                        self._spR.put(0)
                    if self._effL is not None:
                        self._effL.put(0)
                    if self._effR is not None:
                        self._effR.put(0)
                    self._state = S0_IDLE

            self._prev = pressed
            yield self._state