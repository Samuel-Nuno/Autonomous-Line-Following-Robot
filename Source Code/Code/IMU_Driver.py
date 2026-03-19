from pyb import I2C, Pin
import utime


class IMU_Driver:
    """
    BNO055 IMU Driver (I2C)
    - Read chip ID
    - Set fusion mode (IMU / NDOF)
    - Read calibration status
    - Read Euler angles (heading/roll/pitch)
    - Read gyro (angular rate) including yaw rate
    - Read/write calibration coefficients (offsets/radii) for saving/loading
    """

    # -----------------------------
    # I2C Address
    # -----------------------------
    DEFAULT_ADDR = 0x28  # BNO055 default I2C addr (ADR pin low)

    # -----------------------------
    # Core registers
    # -----------------------------
    REG_CHIP_ID   = 0x00
    REG_PAGE_ID   = 0x07

    REG_OPR_MODE  = 0x3D
    REG_PWR_MODE  = 0x3E
    REG_SYS_TRIGGER = 0x3F

    REG_CALIB_STAT = 0x35

    # -----------------------------
    # Operating modes
    # -----------------------------
    MODE_CONFIG = 0x00
    MODE_IMU    = 0x08   # accel+gyro fusion
    MODE_NDOF   = 0x0C   # full fusion (accel+gyro+mag)

    # Power modes
    PWR_NORMAL  = 0x00

    # -----------------------------
    # Euler registers (Page 0, little-endian)
    # 1 deg = 16 LSB
    # -----------------------------
    REG_EUL_HEADING_LSB = 0x1A
    REG_EUL_ROLL_LSB    = 0x1C
    REG_EUL_PITCH_LSB   = 0x1E

    # -----------------------------
    # Gyro registers (Page 0, little-endian)
    # Units: 1 dps = 16 LSB
    # -----------------------------
    REG_GYR_DATA_X_LSB  = 0x14
    # x: 0x14-0x15, y: 0x16-0x17, z: 0x18-0x19

    # -----------------------------
    # Calibration data block (Page 0)
    # Offsets + radii total = 22 bytes starting at 0x55
    # -----------------------------
    REG_CALIB_DATA_START = 0x55
    CALIB_DATA_LEN       = 22

    def __init__(self, i2c, address=DEFAULT_ADDR, rst_pin=None):
        """
        Args:
            i2c: pre-created pyb.I2C object
            address: I2C address (0x28 usually)
            rst_pin: optional pyb.Pin for hardware reset (your table: PC9)
        """
        self._i2c = i2c
        self._addr = address
        self._rst = rst_pin

        utime.sleep_ms(50)  # power-up delay

    # -----------------------------
    # Low-level helpers
    # -----------------------------
    def _read_bytes(self, start_reg, length):
        buf = bytearray(length)
        self._i2c.mem_read(buf, self._addr, start_reg)
        return buf

    def _write_byte(self, reg, val):
        self._i2c.mem_write(val & 0xFF, self._addr, reg)

    def _set_page(self, page):
        self._write_byte(self.REG_PAGE_ID, page & 0x01)
        utime.sleep_ms(2)

    def _bytes_to_int16(self, lsb, msb):
        val = (msb << 8) | lsb
        if val & 0x8000:
            val -= 0x10000
        return val

    # -----------------------------
    # Basic ID / reset / mode
    # -----------------------------
    def read_chip_id(self):
        return self._read_bytes(self.REG_CHIP_ID, 1)[0]

    def hw_reset(self):
        """
        Optional hardware reset if you wired RESET to PC9 (your table shows PC9).
        If you didn't wire reset, you can ignore this.
        """
        if self._rst is None:
            return

        # Active low reset
        self._rst.low()
        utime.sleep_ms(10)
        self._rst.high()
        utime.sleep_ms(650)  # datasheet recommends ~650ms after reset

    def set_mode(self, mode):
        """
        Always switch to CONFIG mode before changing modes (BNO055 requirement).
        """
        self._set_page(0)
        self._write_byte(self.REG_OPR_MODE, self.MODE_CONFIG)
        utime.sleep_ms(25)

        self._write_byte(self.REG_PWR_MODE, self.PWR_NORMAL)
        utime.sleep_ms(10)

        self._write_byte(self.REG_OPR_MODE, mode)
        utime.sleep_ms(25)

    # -----------------------------
    # Calibration
    # -----------------------------
    def get_calib_status(self):
        """
        Returns tuple: (sys, gyro, acc, mag) where each is 0..3
        """
        raw = self._read_bytes(self.REG_CALIB_STAT, 1)[0]
        sys_cal  = (raw >> 6) & 0b11
        gyro_cal = (raw >> 4) & 0b11
        acc_cal  = (raw >> 2) & 0b11
        mag_cal  = (raw >> 0) & 0b11
        return (sys_cal, gyro_cal, acc_cal, mag_cal)

    def read_calibration_data(self):
        self._set_page(0)

        prev_mode = self._read_bytes(self.REG_OPR_MODE, 1)[0]

        self._write_byte(self.REG_OPR_MODE, self.MODE_CONFIG)
        utime.sleep_ms(25)

        data = self._read_bytes(self.REG_CALIB_DATA_START, self.CALIB_DATA_LEN)

        self._write_byte(self.REG_OPR_MODE, prev_mode)
        utime.sleep_ms(25)

        return bytes(data)

    def write_calibration_data(self, calib_bytes):
        if calib_bytes is None or len(calib_bytes) != self.CALIB_DATA_LEN:
            raise ValueError("calib_bytes must be exactly 22 bytes")

        self._set_page(0)
        prev_mode = self._read_bytes(self.REG_OPR_MODE, 1)[0]

        self._write_byte(self.REG_OPR_MODE, self.MODE_CONFIG)
        utime.sleep_ms(25)

        self._i2c.mem_write(bytearray(calib_bytes), self._addr, self.REG_CALIB_DATA_START)
        utime.sleep_ms(10)

        self._write_byte(self.REG_OPR_MODE, prev_mode)
        utime.sleep_ms(25)

    def save_calibration_to_file(self, filename="bno055_calib.bin"):
        """
        Convenience helper for MicroPython filesystem.
        """
        data = self.read_calibration_data()
        with open(filename, "wb") as f:
            f.write(data)

    def load_calibration_from_file(self, filename="bno055_calib.bin"):
        """
        Convenience helper for MicroPython filesystem.
        """
        with open(filename, "rb") as f:
            data = f.read()
        self.write_calibration_data(data)

    # -----------------------------
    # Euler angles
    # -----------------------------
    def read_euler(self):
        """
        Returns (heading_deg, roll_deg, pitch_deg)
        Scaling: 1 deg = 16 LSB
        """
        self._set_page(0)
        buf = self._read_bytes(self.REG_EUL_HEADING_LSB, 6)

        heading_raw = self._bytes_to_int16(buf[0], buf[1])
        roll_raw    = self._bytes_to_int16(buf[2], buf[3])
        pitch_raw   = self._bytes_to_int16(buf[4], buf[5])

        return (heading_raw / 16.0, roll_raw / 16.0, pitch_raw / 16.0)

    def read_heading(self):
        h, _, _ = self.read_euler()
        return h

    # -----------------------------
    # Gyro (angular rates)
    # -----------------------------
    def read_gyro(self):
        """
        Returns (gx_dps, gy_dps, gz_dps)
        Scaling: 1 dps = 16 LSB
        """
        self._set_page(0)
        buf = self._read_bytes(self.REG_GYR_DATA_X_LSB, 6)

        gx_raw = self._bytes_to_int16(buf[0], buf[1])
        gy_raw = self._bytes_to_int16(buf[2], buf[3])
        gz_raw = self._bytes_to_int16(buf[4], buf[5])

        return (gx_raw / 16.0, gy_raw / 16.0, gz_raw / 16.0)

    def read_yaw_rate_dps(self):
        """
        Returns yaw rate (gyro Z) in deg/s.
        """
        _, _, gz = self.read_gyro()
        return gz

    def read_yaw_rate_rads(self):
        """
        Returns yaw rate (gyro Z) in rad/s (often nicer for controls later).
        """
        gz_dps = self.read_yaw_rate_dps()
        return gz_dps * 3.141592653589793 / 180.0
