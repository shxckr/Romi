import time

class IMU:
    """
    BNO055 IMU Driver
    """

    # --- Device Registers ---
    OPR_MODE      = 0x3D
    CALIB_STAT    = 0x35

    EULER_START   = 0x1A
    GYRO_START    = 0x14

    CALIB_START   = 0x55
    CALIB_LEN     = 22

    # --- Modes ---
    CONFIG_MODE   = 0x00
    NDOF          = 0x0C
    IMUPLUS       = 0x08
    COMPASS       = 0x09
    M4G           = 0x0A
    NDOF_FMC_OFF  = 0x0B

    def __init__(self, i2c, addr=0x28):
        """
        Takes in a pyb.I2C object preconfigured in CONTROLLER mode.
        """
        self.i2c = i2c
        self.addr = addr
        self._mode = None

    # -------------------------------------------------
    # MODE CONTROL
    # -------------------------------------------------

    def set_mode(self, mode):
        """
        Change IMU operating mode (fusion modes included).
        """
        # Switch to CONFIG first (datasheet requirement)
        # Because the BNO055 datasheet says you should only change settings 
        # (including switching fusion modes) from CONFIG mode. So this method:
        self.i2c.mem_write(self.CONFIG_MODE, self.addr, self.OPR_MODE)
        time.sleep_ms(20)

        # Switch to desired mode
        self.i2c.mem_write(mode, self.addr, self.OPR_MODE)
        time.sleep_ms(20)

        self._mode = mode

    # -------------------------------------------------
    # CALIBRATION STATUS
    # -------------------------------------------------

    def get_cal_status(self):
        """
        Retrieve and parse calibration status.
        Returns (sys, gyr, acc, mag) each from 0–3.
        """
        buf = bytearray(1)
        self.i2c.mem_read(buf, self.addr, self.CALIB_STAT)
        b = buf[0]

        mag =  b        & 0b11
        acc = (b >> 2)  & 0b11
        gyr = (b >> 4)  & 0b11
        sys = (b >> 6)  & 0b11

        #print("get_cal_status complete")

        return sys, gyr, acc, mag

    # -------------------------------------------------
    # CALIBRATION COEFFICIENTS
    # -------------------------------------------------
    def _read_u8(self, reg):
        buf = bytearray(1)
        self.i2c.mem_read(buf, self.addr, reg)
        return buf[0]

    def get_cal_coeffs(self):
        prev = self._read_u8(self.OPR_MODE)  # read actual chip mode

        self.i2c.mem_write(self.CONFIG_MODE, self.addr, self.OPR_MODE)
        time.sleep_ms(20)

        buf = bytearray(self.CALIB_LEN)
        self.i2c.mem_read(buf, self.addr, self.CALIB_START)

        self.i2c.mem_write(prev, self.addr, self.OPR_MODE)  # always restore
        time.sleep_ms(20)

        return bytes(buf)
    # def get_cal_coeffs(self):
    #     """
    #     Retrieve 22-byte calibration coefficients as binary data.
    #     """
    #     prev = self._mode

    #     # Must be in CONFIG mode
    #     self.i2c.mem_write(self.CONFIG_MODE, self.addr, self.OPR_MODE)
    #     time.sleep_ms(20)

    #     buf = bytearray(self.CALIB_LEN)
    #     self.i2c.mem_read(buf, self.addr, self.CALIB_START)

    #     # Restore previous mode
    #     if prev is not None:
    #         self.i2c.mem_write(prev, self.addr, self.OPR_MODE)
    #         time.sleep_ms(20)

    #     return bytes(buf)

    def set_cal_coeffs(self, coeffs):
        # """
        # Write 22-byte calibration coefficients back to IMU.
        # """
        # if not isinstance(coeffs, (bytes, bytearray)):
        #     raise TypeError("coeffs must be bytes or bytearray")
        # if len(coeffs) != self.CALIB_LEN:
        #     raise ValueError("coeffs must be exactly 22 bytes")

        # prev = self._mode

        # # Enter CONFIG mode
        # self.i2c.mem_write(self.CONFIG_MODE, self.addr, self.OPR_MODE)
        # time.sleep_ms(20)

        # # Write block
        # self.i2c.mem_write(coeffs, self.addr, self.CALIB_START)
        # time.sleep_ms(20)

        # # Restore previous mode
        # if prev is not None:
        #     self.i2c.mem_write(prev, self.addr, self.OPR_MODE)
        #     time.sleep_ms(20)
        if not isinstance(coeffs, (bytes, bytearray)):
            raise TypeError("coeffs must be bytes or bytearray")
        if len(coeffs) != self.CALIB_LEN:
            raise ValueError("coeffs must be exactly 22 bytes")

        prev = self._read_u8(self.OPR_MODE)   # <-- read actual chip mode

        self.i2c.mem_write(self.CONFIG_MODE, self.addr, self.OPR_MODE)
        time.sleep_ms(20)

        self.i2c.mem_write(coeffs, self.addr, self.CALIB_START)
        time.sleep_ms(20)

        self.i2c.mem_write(prev, self.addr, self.OPR_MODE)  # <-- always restore
        time.sleep_ms(20)

    # -------------------------------------------------
    # EULER ANGLES
    # -------------------------------------------------

    def read_euler(self):
        """
        Returns (heading, roll, pitch) in degrees.
        """
        buf = bytearray(6)
        self.i2c.mem_read(buf, self.addr, self.EULER_START)

        h = buf[0] | (buf[1] << 8)
        r = buf[2] | (buf[3] << 8)
        p = buf[4] | (buf[5] << 8)

        # Convert to signed
        if h & 0x8000: h -= 0x10000
        if r & 0x8000: r -= 0x10000
        if p & 0x8000: p -= 0x10000

        # Scale (1 LSB = 1/16 degree)
        return h / 16.0, r / 16.0, p / 16.0

    def heading(self):
        """
        Returns heading (yaw) in degrees.
        """
        buf = bytearray(2)
        self.i2c.mem_read(buf, self.addr, self.EULER_START)

        h = buf[0] | (buf[1] << 8)
        if h & 0x8000:
            h -= 0x10000

        return h / 16.0

    # -------------------------------------------------
    # ANGULAR VELOCITY (GYRO)
    # -------------------------------------------------

    def read_gyro(self):
        """
        Returns (gx, gy, gz) in degrees/second.
        """
        buf = bytearray(6)
        self.i2c.mem_read(buf, self.addr, self.GYRO_START)

        gx = buf[0] | (buf[1] << 8)
        gy = buf[2] | (buf[3] << 8)
        gz = buf[4] | (buf[5] << 8)

        if gx & 0x8000: gx -= 0x10000
        if gy & 0x8000: gy -= 0x10000
        if gz & 0x8000: gz -= 0x10000

        return gx / 16.0, gy / 16.0, gz / 16.0

    def yaw_rate(self):
        # """
        # Returns yaw rate (Z gyro) in degrees/second.
        # """
        # buf = bytearray(6)
        # self.i2c.mem_read(buf, self.addr, self.GYRO_START)

        # gz = buf[4] | (buf[5] << 8)
        # if gz & 0x8000:
        #     gz -= 0x10000

        # return gz / 16.0
        buf = bytearray(2)
        self.i2c.mem_read(buf, self.addr, self.GYRO_START + 4)  # Z LSB at +4
        gz = buf[0] | (buf[1] << 8)
        if gz & 0x8000:
            gz -= 0x10000
        return gz / 16.0