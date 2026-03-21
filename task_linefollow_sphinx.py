'''
This file contains the instructions for Romi's line following task.
'''
try:
    import micropython
except Exception:
    class micropython:
        @staticmethod
        def const(x): return x

try:
    import pyb
except Exception:
    class pyb:
        @staticmethod
        def millis(): return 0

try:
    from task_share import Share, Queue
except Exception:
    class Share: pass
    class Queue:
        def any(self): return False
        def put(self, *a): pass
        def full(self): return False

try:
    from linesensors_sphinx import LineSensors
except Exception:
    class LineSensors: pass

S0_INIT = micropython.const(0)
S1_Wait  = micropython.const(1)
S2_RUN  = micropython.const(2)
S3_CAL  = micropython.const(3)

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
                 sp_left: Share, sp_right: Share, centroidData: Queue,centroidTime: Queue,
                 share_calL, share_calD, collision_mode:Share, yolo_mode:Share, share_lineKp: Share, share_lineKi: Share,
                 DoneCalSh: Share, baseSh: Share):
        '''
        Initializes the task_linefollow which works by using the readings from IR sensors, normalizes the readings according
        to the light and dark calibrations, calculates the centroid of the line as an error distance from the center of the IR 
        sensor array, and setting the appropriate setpoints of the left and right motors according to a Proportional Intetegral 
        controller.
        
        Arguements:
        sensors: an array object of IR sensers
        leftGo: a boolean share that determines when the left motor is allowed to run (high if enabled)
        rightGo: a boolean share that determines when the right motor is allowed to run (high if enabled)
        sp_left: a float share that determines the setpoint of the left motor
        sp_right: a float share that determines the setpoint of the right motor
        centroidData: a float queue that stores the position of the centroid relative to the center of Romi's sensors
        centroidTime: a queue that stores time stamps for centroid data readings
        share_calL: a boolean flag that is high when a request is sent to calibrate the IR sensors for Light readings
        share_calD: a boolean flag that is high when a request is sent to calibrate the IR sensors for Dark readings
        collision_mode: a boolean flag that is high when the motors are controlled by the bumpsensors
        yolo_mode: a boolean flag that is high when the motors are being controled by courseTest Task
        share_lineKp: a float share that holds the proportional gain for line following
        share_lineKi: a float share that holds the integral gain for line following
        DoneCalSh: a boolean share that tells the UI task when calibration is done
        baseSh: a float share that holed the base speed of Romi in mm/s
        
        
        '''

        self._state = S0_INIT
        self.collision_mode = collision_mode    # collision detection flag
        self.yolo_mode = yolo_mode
        self._sensors = sensors                 # IR sensor object
        self._leftGo  = leftGo                  # left motor go flag
        self._rightGo = rightGo                 # right motor go flag
        self._spL = sp_left                     # share for left motor set point
        self._spR = sp_right                    # share for right motor set point
        self._calL = share_calL
        self._calD = share_calD
        self._centroidData = centroidData       # queue for centroid data
        self._centroidTime = centroidTime       # queue for centroid time stamp
        self._share_lineKp = share_lineKp       # share for line follow Kp
        self._share_lineKi = share_lineKi       # share for line follow Ki
        self._DoneCalSh = DoneCalSh
        self.prevt = 0                          # previous time initialized to zero
        self._tRun = 0                          # initial run timestamp
        self.e     = 0                          # error
        self._esum = 0                          # integral of error        
        self.baseSh = baseSh
        self.base = self.baseSh.get()           # 1200  # base speed for line following
        self.max_turn = 400                     # 2000 # maximum turn speed
        self.max_sp = 382                       # 2500 # maximum wheel speed
        self.line_is_dark = True

    @staticmethod
    def _clamp(x, lo, hi):
        '''limits the setpoints of each wheel to remain within the range [lo, hi]'''
        if x < lo: return lo
        if x > hi: return hi
        return x

    def run(self):
        '''
        Runs one iteration of the line following task
        '''
        while True:
            if self.collision_mode.get() == 1:
                # bumper is in charge; do NOT write sp_left/sp_right
                
                pass
            elif self.yolo_mode.get() == 1:
                pass
            elif self._state == S0_INIT:
                # Start stopped
                self._spL.put(0.0)
                self._spR.put(0.0)
                self._state = S1_Wait

            elif self._state == S1_Wait:
                self.base = self.baseSh.get()
                if self._leftGo.get() and self._rightGo.get():
                    self._tRun = pyb.millis()
                    self.e     = 0
                    self._esum = 0
                    prevt = 0
                    self._state = S2_RUN
                elif self._calL.get() or self._calD.get():
                    self._state=S3_CAL
            elif self._state == S2_RUN:
                if self._leftGo.get() and self._rightGo.get():
                    #ADC = self._sensors.read_normalized()
                    self.e = self._sensors.line_error(line_is_dark=self.line_is_dark)
                    t = pyb.millis() - self._tRun
                    dt = t-prevt
                    prevt = t    
                    if not self._centroidData.full():
                        self._centroidData.put(self.e)
                        self._centroidTime.put(t)
                    self._esum += self.e*dt
                    # calculate how much to turn based on PI error and saturation
                    turn = self._clamp(self._share_lineKp.get() * self.e + self._share_lineKi.get() * self._esum, -self.max_turn, self.max_turn)
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
                    self.e = 0
                    self._esum = 0
                    self._state = S1_Wait
                    
            elif self._state == S3_CAL:
                if self._calD.get():
                    self._sensors.calibrateDark()
                    self._calD.put(False)
                    self._DoneCalSh.put(True)
                    self._state = S1_Wait
                elif self._calL.get():
                    self._sensors.calibrateLight()
                    self._calL.put(False)
                    self._DoneCalSh.put(True)
                    self._state = S1_Wait
            yield self._state
