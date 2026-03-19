import micropython
from pyb import Pin, Timer
from time import ticks_us, ticks_diff

from motor import Motor
from encoder import Encoder
from task_share import Share, Queue

S0_WAIT = micropython.const(0)
S1_RUN  = micropython.const(1)


class task_right_drive:

    def __init__(self,
                 motor_tim: Timer,
                 goFlag: Share,
                 effR: Share,
                 velR: Share,
                 dataValuesR: Queue,
                 timeValuesR: Queue,
                 posR_share=None):

        self._state = S0_WAIT

        self._mot = Motor(PWM=Pin.cpu.C6, DIR=Pin.cpu.H0, nSLP=Pin.cpu.H1,
                          tim=motor_tim, channel=1)
        self._mot.enable()

        self._enc = Encoder(tim=3, chA_pin=Pin.cpu.B4, chB_pin=Pin.cpu.B5)

        self._goFlag = goFlag
        self._effR = effR
        self._velR = velR
        self._posR = posR_share

        self._dataValues = dataValuesR
        self._timeValues = timeValuesR
        self._t0 = 0

    def run(self):
        while True:
            self._enc.update()
            pos = self._enc.get_position()
            v = self._enc.get_velocity()

            self._velR.put(v)
            if self._posR is not None:
                self._posR.put(pos)

            if self._goFlag.get():
                u = self._effR.get()
                try:
                    u = int(u)
                except Exception:
                    pass
                self._mot.set_effort(u)
            else:
                self._mot.set_effort(0)
                self._effR.put(0)

            if self._state == S0_WAIT:
                if self._goFlag.get():
                    self._t0 = ticks_us()
                    self._state = S1_RUN

            elif self._state == S1_RUN:
                if not self._goFlag.get():
                    self._state = S0_WAIT

                if not (self._dataValues.full() or self._timeValues.full()):
                    t = ticks_us()
                    self._timeValues.put(ticks_diff(t, self._t0))
                    self._dataValues.put(v)

            yield self._state