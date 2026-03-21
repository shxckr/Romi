''' This file defines a class used to create and manage encoder objects.
'''
try:
    from time import ticks_us, ticks_diff
except ImportError:
    def ticks_us():
        return 0

    def ticks_diff(a, b):
        return a - b

try:
    import pyb
except ImportError:
    class _DummyPin:
        def __init__(self, *args, **kwargs):
            pass

    class _DummyChannel:
        def __init__(self, *args, **kwargs):
            pass

    class _DummyTimer:
        ENC_AB = 0

        def __init__(self, *args, **kwargs):
            self._counter = 0
            self._period = 65535

        def period(self):
            return self._period

        def counter(self):
            return self._counter

        def channel(self, *args, **kwargs):
            return _DummyChannel()

    class _DummyPyb:
        Pin = _DummyPin
        Timer = _DummyTimer

    pyb = _DummyPyb()

import math

class encoder:
    ''' A class that can be used to instantiate encoder objects.
    '''
    
    #def __init__(self):
    def __init__(self, tim, chA_pin, chB_pin):
        """Initialize the encoder object.
        """
        
        #: Internal position accumulator placeholder.
        self._position  = 0
        #: Total accumulated encoder count in ticks since the last zero().
        self.position   = 0     # Total accumulated position of the encoder
         #: Most recent raw timer counter value.
        self.prev_count = 0     # Counter value from the most recent update
        #: Change in encoder counts measured during the last update().
        self.delta      = 0     # Change in count between last two updates
        #: Time interval in seconds between the two most recent update() calls.
        self.dt         = 0     # Amount of time between last two updates
        

        #: Hardware timer configured in encoder mode.
        self.tim = pyb.Timer(tim, period=0xFFFF, prescaler=0)
        #: Timer auto-reload period value.
        self._period = self.tim.period()
        #: Total number of timer counts in one full counter cycle.
        self._counts_per_cycle = self._period + 1   ## Ar+1
        #: Half of the counter range, used for overflow and underflow correction.
        self._half_range = self._counts_per_cycle // 2  ## (Ar+1)/2
        # -----------------------------------------------

        #: Pin connected to encoder channel A.
        self.chA_pin = chA_pin if isinstance(chA_pin, pyb.Pin) else pyb.Pin(chA_pin)
        #: Pin connected to encoder channel B.
        self.chB_pin = chB_pin if isinstance(chB_pin, pyb.Pin) else pyb.Pin(chB_pin)

        self.tim.channel(1, pyb.Timer.ENC_AB, pin=self.chA_pin)
        self.tim.channel(2, pyb.Timer.ENC_AB, pin=self.chB_pin)

         # Initialize previous values
        self.prev_count = self.tim.counter()
        #: Timestamp in microseconds of the previous update() call.
        self._prev_time = ticks_us()


    
    def update(self, cbSRC = None):
        ''' Update the encoder count. This function is meant to be called
            periodically in a task or using a Timer
        '''
        now = ticks_us()
        dt_us = ticks_diff(now, self._prev_time)
        self._prev_time = now

        # Guard against invalid dt
        if dt_us <= 0:
            return

        self.dt = dt_us / 1_000_000  # seconds

        # Read current counter
        count = self.tim.counter()
        raw_delta = count - self.prev_count

        # based on class lecture....Counter reload correction (no hard-coded limits)
        if raw_delta > self._half_range: # if delta greater than (ar+1)/2 then underflow
            raw_delta -= self._counts_per_cycle
        elif raw_delta < -self._half_range:   # if delta is less than - (ar+1)/2 then overflow
            raw_delta += self._counts_per_cycle

        # Update state
        self.delta = raw_delta
        self.position += self.delta
        self.prev_count = count
    def get_position(self):
        ''' Returns the current position of the encoder in units of ticks
        '''
        return self.position * (math.pi * 70) / 1440
    def get_velocity(self):
        '''Returns a measure of velocity using the the most recently updated
           value of delta as determined within the update() method'''
        return self.delta / self.dt * (math.pi*70)/(1440)
        
    
    def zero(self):
        ''' Zeros the encoder position at the current orientation. Used to
            reestablish a new datum position for the encoder
        '''
        self.position = 0
        self.delta = 0
        self.prev_count = self.tim.counter()
        self._prev_time = ticks_us()
        self.dt = 0