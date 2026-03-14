import pyb
import utime

class LineSensors:
    def __init__(self, IN_PINS: list(str), samples=4):
        self.IN_PINS = IN_PINS
        self.adcs = [pyb.ADC(pyb.Pin(pin)) for pin in IN_PINS]
        self.samples = max(1, int(samples))
        # self.mins = [1500] * len(self.IN_PINS)
        # self.maxs = [2500] * len(self.IN_PINS)
        self.mins = [0] * len(self.IN_PINS)
        self.maxs = [4095] * len(self.IN_PINS)
        # line sensors read lower numbers for white

    def read_raw(self):
        vals = []
        for adc in self.adcs:
            s = 0
            for _ in range(self.samples):
                s += adc.read()          # typically 0..4095
            vals.append(s // self.samples)
        return tuple(vals)
            
    def calibrateDark(self):
        # --- Dark calibration ---
        sample_number=100
        self.maxs = [4095] * len(self.IN_PINS)  # reset maxs before calibration
        for _ in range(sample_number):
            vals = self.read_raw()   # tuple of 7 values
            for i in range(len(self.IN_PINS)):
                self.maxs[i] = min(self.maxs[i], vals[i])
        #print("Dark calibration complete")
        # self.maxs = [1908, 1706, 1725, 1725, 1725, 1369, 1807]
        print(f"maxs: {self.maxs}")
    def _getDark(self):
        return self.maxs
    
    def calibrateLight(self):
            sample_number=100
            self.mins = [0] * len(self.IN_PINS)  # reset mins before calibration
            for _ in range(sample_number):
                vals = self.read_raw()
                for i in range(len(self.IN_PINS)):
                    self.mins[i] = max(self.mins[i], vals[i])
            #print("Light calibration complete")
            print(f"mins: {self.mins}")
    
    def _getLight(self):
        return self.mins
    
    def _normalize(self, val, vmin, vmax):
        span = max(1, vmax - vmin)
        x = (val - vmin) / span
        if x < 0.0: x = 0.0
        if x > 1.0: x = 1.0
        return x

    def read_normalized(self):
        raw = self.read_raw()
        #print(raw) # tried to print?
        return [self._normalize(raw[i], self.mins[i], self.maxs[i]) for i in range(len(self.IN_PINS))]

    def line_error(self, line_is_dark=True):
        norm = self.read_normalized()
        #print(norm)

        # convert to "line strength" (bigger => more on the line)
        if line_is_dark:
            strengths = [1.0 - v for v in norm]
        else:
            strengths = norm
        # print(strengths)

        total = sum(strengths)
        if total < 1e-6:
            return 0.0  # lost line; could also return last error

        # positions across sensor bar: -1 .. +1
        pos_weights = [(i - 3) / 3.0 for i in range(len(self.IN_PINS))]
        pos = sum(w * s for w, s in zip(pos_weights, strengths)) / total
        #print(pos)
        return pos
    


    # Yellow pins = ['PA4', 'PB0', 'PB1', 'PC0', 'PC1', 'PC2', 'PC3']
    # Pink pins = ['PC2','PC3','PC0',"PC1","PB0","PA4","PC_4"]