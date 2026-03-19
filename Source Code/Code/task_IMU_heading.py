import micropython
from time import sleep_ms
from pyb import I2C, Pin
from IMU_Driver import IMU_Driver

S0_INIT = micropython.const(0)
S1_RUN  = micropython.const(1)


class task_IMU_heading:
    """
        psi_share      -> relative heading in radians
        psi_dot_share  -> yaw rate in rad/s
        Call zero_heading() whenever you want the current robot
        heading to become psi = 0.
    """

    def __init__(self,
                 psi_share,
                 psi_dot_share,
                 imu_mode=IMU_Driver.MODE_IMU,
                 imu_reset_pin=Pin.cpu.C9):

        self._state = S0_INIT

        self._psi_share = psi_share
        self._psi_dot_share = psi_dot_share

        self._imu_mode = imu_mode
        self._imu_rst = Pin(imu_reset_pin, Pin.OUT_PP)
        self._imu_rst.high()

        self._imu = None
        self._psi0 = 0.0

    def zero_heading(self):
        # Set current absolute heading as zero reference
        try:
            if self._imu is not None:
                psi_abs = float(self._imu.read_heading()) * (3.141592653589793 / 180.0)
                self._psi0 = psi_abs
                self._psi_share.put(0.0)
        except Exception:
            pass

    def run(self):
        while True:
            if self._state == S0_INIT:
                # Keep reset pin high before IMU init
                self._imu_rst.high()
                sleep_ms(10)

                i2c = I2C(1, I2C.CONTROLLER, baudrate=100000)
                self._imu = IMU_Driver(i2c)

                # Set fusion mode
                self._imu.set_mode(self._imu_mode)

                # Establish initial zero
                psi_abs = float(self._imu.read_heading()) * (3.141592653589793 / 180.0)
                self._psi0 = psi_abs

                self._psi_share.put(0.0)
                self._psi_dot_share.put(0.0)

                self._state = S1_RUN

            elif self._state == S1_RUN:
                try:
                    psi_abs = float(self._imu.read_heading()) * (3.141592653589793 / 180.0)
                    psi = psi_abs - self._psi0
                    psi_dot = float(self._imu.read_yaw_rate_rads())

                    self._psi_share.put(psi)
                    self._psi_dot_share.put(psi_dot)

                except Exception:
                    # Keep old values if IMU hiccups
                    pass

            yield self._state
