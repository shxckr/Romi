''' This file implements a "dummy" class to use in place of motor driver objects
'''
from pyb import Pin, Timer
class motor_driver:
    ''' A dummy class that can be instantiated inplace of motor driver objects
    '''
    
    #def __init__(self):
    def __init__(self, tim, chan, PWM, DIR, nSLP):
        ''' Initializes a motor driver object'''
        # print("Motor object instantiated")
        self.nSLP_pin = Pin(nSLP, mode=Pin.OUT_PP, value=0)
        self.DIR_pin  = Pin(DIR,  mode=Pin.OUT_PP)

        # Make PWM a Pin here (works whether PWM is 'PA8' or Pin('PA8'))
        self.PWM_pin = Pin(PWM)

        # Use the standard stm32 API
        self.PWM_chan = tim.channel(chan, Timer.PWM, pin=self.PWM_pin, pulse_width_percent=0)

        self.enabled = False
        pass
    
    def enable(self):
        ''' Enables/wakes up a motor driver'''
        # print("Enabling motor")
        self.nSLP_pin.high()
        self.PWM_chan.pulse_width_percent(0)  # brake
        self.enabled = True
        #pass
        pass
    
    def disable(self):
        ''' Disables/puts to sleep a motor driver'''
        # print("Disabling motor")
        self.PWM_chan.pulse_width_percent(0)
        self.nSLP_pin.low()
        self.enabled = False
        pass
    
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

        # if not self.enabled:
        #     return
        # if eff > 0 :
        #     self.DIR_pin.low()
        #     self.PWM_chan.pulse_width_percent(eff)
        # elif eff < 0:
        #     self.DIR_pin.high()
        #     self.PWM_chan.pulse_width_percent(-eff)
        # else:
        #     self.PWM_chan.pulse_width_percent(0)
        # pass

        if not self.enabled:
            return
        if eff > 0 :
            # self.DIR_pin.high()
            # self.PWM_chan.pulse_width_percent(eff)
        #if eff < 0 :
            self.DIR_pin.low()
            self.PWM_chan.pulse_width_percent(eff)
        elif eff < 0:
            # self.DIR_pin.low()
            # self.PWM_chan.pulse_width_percent(-eff)
        #elif eff > 0:
            self.DIR_pin.high()
            self.PWM_chan.pulse_width_percent(-eff)
        else:
            self.PWM_chan.pulse_width_percent(0)
        pass
        # print(f"Setting motor effort to {effort}%")
        #pass