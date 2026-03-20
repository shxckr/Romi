try:
    import pyb
    import utime
except ImportError:
    class _DummyPin:
        def __init__(self, *args, **kwargs):
            pass

    class _DummyADC:
        def __init__(self, *args, **kwargs):
            pass

        def read(self):
            return 0

    class _DummyChannel:
        def pulse_width_percent(self, *args, **kwargs):
            pass

    class _DummyPyb:
        Pin = _DummyPin
        ADC = _DummyADC

    pyb = _DummyPyb()

    class _DummyUtime:
        @staticmethod
        def sleep_ms(ms):
            pass

    utime = _DummyUtime()

class LineSensors:
    """Driver for a line sensor array using ADC readings."""
    def __init__(self, IN_PINS, samples=4):
        """Initialize the line sensor array.

        Args:
            IN_PINS: List of pin names connected to the line sensors.
            samples: Number of ADC readings to average for each sensor
                to reduce noise.
        """
        #: List of microcontroller pin names connected to the line sensors.
        self.IN_PINS = IN_PINS
         #: ADC objects, one for each line sensor input pin.
        self.adcs = [pyb.ADC(pyb.Pin(pin)) for pin in IN_PINS]
        #: Number of ADC samples averaged per sensor reading.
        self.samples = max(1, int(samples))
        # Calibration values used for normalization.
        # Lower readings correspond to the light background.
        # Higher readings correspond to the dark line.
        #: Calibration readings for the light surface, one per sensor.
        self.mins = [970, 866, 762, 324, 322, 382, 683]
        #: Calibration readings for the dark surface, one per sensor.
        self.maxs = [3309, 3344, 3128, 3203, 3146, 3305, 3303]
        # line sensors read lower numbers for white
        
        # Light Calibration: [970, 866, 762, 324, 322, 382, 683]
        # Dark Calibration: [2780, 2532, 2428, 2417, 2420, 2188, 2416]

    def read_raw(self):
        """Read averaged raw ADC values from all sensors.

        Returns:
            A tuple containing the averaged ADC reading from each sensor.
        """
        vals = []
        for adc in self.adcs:
            s = 0
            for _ in range(self.samples):
                s += adc.read()          # typically 0..4095
            vals.append(s // self.samples)
        return tuple(vals)
            
    def calibrateDark(self):
        """Calibrate the sensors over the dark line surface.

        This method records the highest observed readings for each sensor,
        which are used as the dark-reference values for normalization.
        """
        # --- Dark calibration ---
        sample_number=100
        self.maxs = [4095] * len(self.IN_PINS)  # reset maxs before calibration
        for _ in range(sample_number):
            vals = self.read_raw()   # tuple of 7 values
            for i in range(len(self.IN_PINS)):
                self.maxs[i] = min(self.maxs[i], vals[i])
        # self.maxs = [1908, 1706, 1725, 1725, 1725, 1369, 1807]
        print(f"maxs: {self.maxs}")
        
    def _getDark(self):
        """Return the current dark calibration values (maxs)."""
        return self.maxs
    
    def calibrateLight(self):
        """Calibrate the sensors over the light surface."""
        sample_number = 100
        self.mins = [0] * len(self.IN_PINS)

        for _ in range(sample_number):
            vals = self.read_raw()
            for i in range(len(self.IN_PINS)):
                self.mins[i] = max(self.mins[i], vals[i])

        print(f"mins: {self.mins}")
    
    def _getLight(self):
        """Return the current light calibration values (mins)."""
        return self.mins
    
    def _normalize(self, val, vmin, vmax):
        """Normalize one sensor reading to the range 0.0 to 1.0.
        Args:
            val: Raw ADC reading.
            vmin: Calibration reading for the light surface.
            vmax: Calibration reading for the dark surface.

        Returns:
            A float in the range [0.0, 1.0], where 0.0 corresponds to the
            light calibration and 1.0 corresponds to the dark calibration.
        """
        span = max(1, vmax - vmin)
        x = (val - vmin) / span
        if x < 0.0: x = 0.0
        if x > 1.0: x = 1.0
        return x

    def read_normalized(self):
        """Read normalized values from all sensors.

        Returns:
            A list of normalized sensor values in the range [0.0, 1.0],
            where 0.0 represents the light background and 1.0 represents
            the dark line.
        """
        raw = self.read_raw()
        return [self._normalize(raw[i], self.mins[i], self.maxs[i]) for i in range(len(self.IN_PINS))]

    def line_error(self, line_is_dark=True):
        """Compute the line position error from the normalized readings.

        Args:
            line_is_dark: True if the line is darker than the background.
                False if the line is lighter than the background.

        Returns:
            A value in the range [-1.0, 1.0], where 0.0 means the line is
            centered, negative means the line is left of center, and
            positive means the line is right of center.
        """
        norm = self.read_normalized()
        # convert to "line strength" (bigger => more on the line)
        if line_is_dark:
            strengths = [1.0 - v for v in norm]
        else:
            strengths = norm
        total = sum(strengths)
        if total < 1e-6:
            return 0.0  # lost line; could also return last error

        # positions across sensor bar: -1 .. +1
        pos_weights = [(i - 3) / 3.0 for i in range(len(self.IN_PINS))]
        pos = sum(w * s for w, s in zip(pos_weights, strengths)) / total
        return pos
    
    def set_emitter_power(self, percent):
        """Set the IR emitter power as a percentage from 0 to 100.

        Args:
            percent: Desired emitter power percentage.
        """
        """Set the power of the IR emitters as a percentage (0-100%)."""
        percent = max(0, min(100, percent))
        self.led_ch.pulse_width_percent(percent)



    # Yellow pins = ['PA4', 'PB0', 'PB1', 'PC0', 'PC1', 'PC2', 'PC3']