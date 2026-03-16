''' This file demonstrates an example motor task using a custom class with a
    run method implemented as a generator
'''
from motor_driver import motor_driver
from encoder      import encoder
from task_share   import Share, Queue
#rom task_user    import task_user
from utime        import ticks_us, ticks_diff
import micropython
import math

S0_INIT = micropython.const(0) # State 0 - initialiation
S1_WAIT = micropython.const(1) # State 1 - wait for go command
S2_RUN  = micropython.const(2) # State 2 - run closed loop control

class task_motor:
    '''
    A class that represents a motor task. The task is responsible for reading
    data from an encoder, performing closed loop control, and actuating a motor.
    Multiple objects of this class can be created to work with multiple motors
    and encoders.
    '''

    def __init__(self,
             mot: motor_driver, enc: encoder,
             goFlag: Share, share_kp: Share, share_ki: Share,
             share_setpoint: Share,
             share_effort: Share, db_share):
        '''
        Initializes a motor task object
        
        Args:
            mot (motor_driver): A motor driver object
            enc (encoder):      An encoder object
            goFlag (Share):     A share object representing a boolean flag to
                                start data collection
            dataValues (Queue): A queue object used to store collected encoder
                                position values
            timeValues (Queue): A queue object used to store the time stamps
                                associated with the collected encoder data
        '''

        self._state: int        = S0_INIT    # The present state of the task       
        
        self._mot: motor_driver = mot        # A motor object
        
        self._enc: encoder      = enc        # An encoder object
        
        self._goFlag: Share     = goFlag     # A share object representing a
                                             # flag to start data collection
        self._share_kp: Share   = share_kp
        self._share_ki: Share   = share_ki
        self._share_effort = share_effort

        self._share_setpoint: Share  = share_setpoint
        
        self._db_share: Share = db_share
        self._startTime: int    = 0          # The start time (in microseconds)
                                             # for a batch of collected data
        self._Kp: float = 0.05
        self._Ki: float = 0.0 # small value!!
        self._setpoint: float = 3000
        self._e_prev: float = 0.0
        self._e_int: float = 0.0
        self._vel_filt = 0.0
        self._vel_alpha = 0.4
        self._deadband = 15.0        # minimum PWM to overcome friction
        self._deadband_hyst = 10.0
        self._vel_prev = 0.0
        
        print("Motor Task object instantiated")
        
    def run(self):
        '''
        Runs one iteration of the task
        '''
        
        while True:
            
            if self._state == S0_INIT: # Init state (can be removed if unneeded)
                self._state = S1_WAIT
                
            elif self._state == S1_WAIT: # Wait for "go command" state
                if self._goFlag.get():   
                    # Captur start time and reset errors
                    self._startTime = ticks_us()
                    self._e_prev = 0.0
                    self._e_int = 0.0
                    self._mot.enable()   
                    self._state = S2_RUN
                
            elif self._state == S2_RUN: # Closed-loop control state
               
                if not self._goFlag.get():
                    self._mot.set_effort(0)
                    self._mot.disable()
                    self._share_effort.put(0)
                    
                    self._e_int = 0.0
                    self._state = S1_WAIT
                    yield self._state
                # Update encoder and get velocity
                self._enc.update()
                vel_raw = self._enc.get_velocity()
                dt = self._enc.dt
                if dt <= 0:
                    yield self._state
                 
                self._vel_filt = (self._vel_alpha * vel_raw) + ((1 - self._vel_alpha) * self._vel_filt)

                 # controller
                self._kp = self._share_kp.get()
                self._ki = self._share_ki.get()
                self._setpoint = self._share_setpoint.get()
                e = self._setpoint - self._vel_filt
                self._e_int += e*dt
                self._e_int = max(min(self._e_int, 200), -200)
                self._vel_prev = self._vel_filt
                self._e_prev = e

                effort = (self._kp*e) + (self._ki*self._e_int)
               

                # --- TEMPORARY: no deadband ---
                out = max(min(effort, 100), -100)
                self._mot.set_effort(out)
                self._share_effort.put(out)

                 # Collect a timestamp to use for this sample
                t   = ticks_us()

                if not self._goFlag.get():
                    self._mot.set_effort(0)
                    self._mot.disable()
                    self._state = S1_WAIT
                    yield self._state
                
            yield self._state