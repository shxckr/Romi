''' This file implements a "dummy" class to use in place of encoder objects
'''
from time import ticks_us, ticks_diff   # Use to get dt value in update()
#from pyb import Timer
import pyb
#from random import random
import math

class encoder:
    ''' A dummy class that can be instantiated in place of encoder objects
    '''
    
    #def __init__(self):
    def __init__(self, tim, chA_pin, chB_pin):
        ''' Initializes an encoder object '''
        # print("Encoder object instantiated")

        ############################### og code
        #self.zero()
        self._position = 0
        ######################################## old code ########################
        self.position   = 0     # Total accumulated position of the encoder
        self.prev_count = 0     # Counter value from the most recent update
        self.delta      = 0     # Change in count between last two updates
        self.dt         = 0     # Amount of time between last two updates
        

        self.tim = pyb.Timer(tim, period=0xFFFF, prescaler=0)

           # --- what you were missing for non-hard-coded wraparound ---
        self._period = self.tim.period()
        self._counts_per_cycle = self._period + 1   ## Ar+1
        self._half_range = self._counts_per_cycle // 2  ## (Ar+1)/2
        # -----------------------------------------------

        self.chA_pin = chA_pin if isinstance(chA_pin, pyb.Pin) else pyb.Pin(chA_pin)
        self.chB_pin = chB_pin if isinstance(chB_pin, pyb.Pin) else pyb.Pin(chB_pin)

        self.tim.channel(1, pyb.Timer.ENC_AB, pin=self.chA_pin)
        self.tim.channel(2, pyb.Timer.ENC_AB, pin=self.chB_pin)

         # Initialize previous values
        self.prev_count = self.tim.counter()
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
        
        # print("Encoder updated")
        # self._position += int(10*(random()-0.5))
        pass
    def get_position(self):
        ''' Returns the current position of the encoder
        
        Returns:
            int The current position of the encoder in units of ticks
        '''
        #self.mm_per_tick = (math.pi * 70) / 1440
        return self.position * (math.pi * 70) / 1440
        #return self.position
    
    
    def get_velocity(self):
        '''Returns a measure of velocity using the the most recently updated
           value of delta as determined within the update() method'''
        return self.delta / self.dt * (math.pi*70)/(1440)
        
    
    def zero(self):
        ''' Zeros the encoder position at the current orientation. Used to
            reestablish a new datum position for the encoder
        '''
        # print("Encoder position zeroed")
        self.position = 0
        self.delta = 0
        self.prev_count = self.tim.counter()
        self._prev_time = ticks_us()
        self.dt = 0
        pass