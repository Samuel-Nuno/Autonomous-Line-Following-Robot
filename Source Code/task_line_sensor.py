import micropython
from task_share import Share
from linesensor import linesensor

S0_INIT = micropython.const(0)
S1_RUN  = micropython.const(1)

class task_line_sensor:
    # Periodically read the QTR-MD-08A array and publish a centroid in [0..7].
    
    def __init__(self, adc_pins, centroid_share: Share, enable_share: Share):
        # Create sensor object
        self._sensor = linesensor(adc_pins)
        self.sensor = self._sensor   
        self._c_share = centroid_share
        self._en_share = enable_share

        self._state = S0_INIT

    def run(self):
        while True:
            if self._state == S0_INIT:
                self._c_share.put(-1.0)   # start with "no line"
                self._state = S1_RUN

            elif self._state == S1_RUN:
                # Only bother reading if line following is enabled
                if self._en_share.get() != 0:
                    c = self._sensor.centroid()
                    if c is None:
                        # No line detected – use value -1
                        self._c_share.put(-1.0)
                    else:
                        self._c_share.put(float(c))
                # If disabled, we just leave the last value alone

            yield self._state