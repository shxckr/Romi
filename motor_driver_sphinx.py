''' This file implements a  class to use in place of motor driver objects
'''
try:
    from pyb import Pin, Timer
except Exception:
    class Pin:
        OUT_PP = 0
        def __init__(self, *a, **k): pass
        def high(self): pass
        def low(self): pass

    class Timer:
        PWM = 0
        def __init__(self, *a, **k): pass
        def channel(self, *a, **k):
            class C:
                def pulse_width_percent(self, *a): pass
            return C()

class motor_driver:
    ''' A class that can be instantiated inplace of motor driver objects
    '''
    
    def __init__(self, tim, chan, PWM, DIR, nSLP):
        ''' Initializes a motor driver object'''
        #: Sleep-control output pin for enabling or disabling the motor driver.
        self.nSLP_pin = Pin(nSLP, mode=Pin.OUT_PP, value=0)
        #: Direction-control output pin for setting motor rotation direction.
        self.DIR_pin  = Pin(DIR,  mode=Pin.OUT_PP)
        # Make PWM a Pin here (works whether PWM is 'PA8' or Pin('PA8'))
        #: PWM output pin connected to the motor driver's speed input.
        self.PWM_pin = Pin(PWM)
        # Use the standard stm32 API
        #: Timer channel configured for PWM duty-cycle control.
        self.PWM_chan = tim.channel(chan, Timer.PWM, pin=self.PWM_pin, pulse_width_percent=0)
        #: True when the motor driver is enabled and allowed to drive the motor.
        self.enabled = False
    
    def enable(self):
        ''' Enables/wakes up a motor driver'''
        self.nSLP_pin.high()
        self.PWM_chan.pulse_width_percent(0)  # brake
        self.enabled = True
    
    def disable(self):
        ''' Disables/puts to sleep a motor driver'''
        self.PWM_chan.pulse_width_percent(0)
        self.nSLP_pin.low()
        self.enabled = False
    
    def set_effort(self, effort):
        ''' Sets the effort of a motor driver
        
        Args:
            effort (float): The desired motor effort as a signed percentage
                            (+/- 100%)
        '''
        try:
            eff = float(effort)
        except Exception:
            return   # or raise ValueError if your instructor wants
        ### sets limit between - 100 to 100
        if eff < -100 or eff > 100:
            return   # or raise ValueError
        if not self.enabled:
            return
        if eff > 0 :
            self.DIR_pin.low()
            self.PWM_chan.pulse_width_percent(eff)
        elif eff < 0:
            self.DIR_pin.high()
            self.PWM_chan.pulse_width_percent(-eff)
        else:
            self.PWM_chan.pulse_width_percent(0)