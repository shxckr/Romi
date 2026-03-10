import micropython
from task_share import Share, Queue
from linesensors import LineSensors
import pyb

S0_INIT = micropython.const(0)
S1_CAL  = micropython.const(1)
S2_RUN  = micropython.const(2)

class task_linefollow:
    """
    Line-follow task:
      e = sensors.line_error() in ~[-1, +1]
      turn = Kline * e
      spL = base - turn
      spR = base + turn
    Writes spL/spR into Shares used by motor controllers.
    """

    def __init__(self,
                 sensors: LineSensors,
                 leftGo: Share, rightGo: Share,
                 sp_left: Share, sp_right: Share, centroidData: Queue,centroidTime: Queue):

        self._state = S0_INIT

        self._sensors = sensors
        self._leftGo  = leftGo
        self._rightGo = rightGo

        self._spL = sp_left
        self._spR = sp_right
        self._centroidData = centroidData
        #self._centroidData = centroidData
        self._centroidTime = centroidTime
   # self._t0 = pyb.millis()
        self._t0 = pyb.millis()

        # Tune these to your robot / motor units
        # self.base = 2500.0       # (example) ticks/s or whatever your PI expects
        # self.Kline = 900.0       # how aggressively you steer per unit error
        # self.max_turn = 2000.0   # clamp differential command
        # self.max_sp = 4000.0     # clamp absolute setpoint
        self.base = 1200
        self.Kline = 3500
        self.max_turn = 2000
        self.max_sp = 2500

        self.line_is_dark = True
        self.cal_ms = 1500

        # Optional: remember last valid error for lost-line behavior
        self._last_e = 0.0

    @staticmethod
    def _clamp(x, lo, hi):
        if x < lo: return lo
        if x > hi: return hi
        return x

    def run(self):
        while True:

            if self._state == S0_INIT:
                # Start stopped
                self._spL.put(0.0)
                self._spR.put(0.0)
                self._state = S1_CAL

            elif self._state == S1_CAL:
                # Blocking calibration (OK if your scheduler can tolerate it)
                # Move robot so sensors see both line and background during this time.
                # self._sensors.calibrate(sample_number=100, sample_period_ms=10)
                self._state = S2_RUN

            elif self._state == S2_RUN:
                if self._leftGo.get() and self._rightGo.get():
                    #ADC = self._sensors.read_normalized()
                    e = self._sensors.line_error(line_is_dark=self.line_is_dark)
                    t = pyb.millis() - self._t0
                        
                    if not self._centroidData.full():
                        self._centroidData.put(e)
                        #print(f"yo this centroid {e} and this time {pyb.millis()}")
                        self._centroidTime.put(t)
                    # If you want: keep last error when line is lost
                    # (Our LineSensors returns 0.0 on lost line. If that’s ambiguous for you,
                    # change LineSensors to return None when lost and handle it here.)
                    self._last_e = e

                    turn = self._clamp(self.Kline * e, -self.max_turn, self.max_turn)

                    spL = self.base - turn
                    spR = self.base + turn

                    spL = self._clamp(spL, -self.max_sp, self.max_sp)
                    spR = self._clamp(spR, -self.max_sp, self.max_sp)

                    self._spL.put(spL)
                    self._spR.put(spR)
                else:
                    # Disabled → stop
                    self._spL.put(0.0)
                    self._spR.put(0.0)

            yield self._state