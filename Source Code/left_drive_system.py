import micropython
from pyb import Pin, Timer
from time import ticks_us, ticks_diff

from motor import Motor
from encoder import Encoder
from task_share import Share, Queue

S0_WAIT = micropython.const(0)
S1_RUN  = micropython.const(1)


class task_left_drive:

    def __init__(self, motor_tim: Timer, goFlag: Share, effL: Share, velL: Share,
                 dataValuesL: Queue,
                 timeValuesL: Queue,
                 posL_share=None,
                 lf_centroid_share=None,
                 lf_centroid_queue=None):

        self._state = S0_WAIT

        self._mot = Motor(PWM=Pin.cpu.C7, DIR=Pin.cpu.B7, nSLP=Pin.cpu.C13,
                          tim=motor_tim, channel=2)
        self._mot.enable()

        self._enc = Encoder(tim=1, chA_pin=Pin.cpu.A8, chB_pin=Pin.cpu.A9)

        self._goFlag = goFlag
        self._effL = effL
        self._velL = velL
        self._posL = posL_share

        self._dataValues = dataValuesL
        self._timeValues = timeValuesL
        self._t0 = 0

        self._lf_c_share = lf_centroid_share
        self._lf_c_queue = lf_centroid_queue

    def run(self):
        while True:
            self._enc.update()
            pos = self._enc.get_position()
            v = self._enc.get_velocity()

            self._velL.put(v)
            if self._posL is not None:
                self._posL.put(pos)

            if self._goFlag.get():
                u = self._effL.get()
                try:
                    u = int(u)
                except Exception:
                    pass
                self._mot.set_effort(u)
            else:
                self._mot.set_effort(0)
                self._effL.put(0)

            if self._state == S0_WAIT:
                if self._goFlag.get():
                    self._t0 = ticks_us()
                    self._state = S1_RUN

            elif self._state == S1_RUN:
                if not self._goFlag.get():
                    self._state = S0_WAIT

                else:
                    if not (self._dataValues.full() or self._timeValues.full()):
                        t = ticks_us()
                        self._timeValues.put(ticks_diff(t, self._t0))
                        self._dataValues.put(v)

                        if (self._lf_c_share is not None) and (self._lf_c_queue is not None):
                            if not self._lf_c_queue.full():
                                self._lf_c_queue.put(self._lf_c_share.get())

            yield self._state