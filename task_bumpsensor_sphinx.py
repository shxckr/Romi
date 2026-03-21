try:
    import micropython
except Exception:
    class micropython:
        @staticmethod
        def const(x): return x

try:
    import pyb
except Exception:
    pass

S0_INIT = micropython.const(0)
S1_WAIT = micropython.const(1)

class task_bumpsensor:
    '''
    This class allows easy integration of an array of bump sensors seperated into left and right sides. It 
    creates a bump sensor object that raises a flag to tell other tasks when a bump sensor is hit. 
    
    Arguements:
    left_pins: list of pins connected to the left bump sensors on Romi
    right_pins: list of pins connected to the right bump sensors on Romi
    sp_left: a share that allows the bump sensors to alter the set point of the left motor
    sp_right: a share that allows the bump sensors to alter the set point of the right motor
    collition_mode: a share that acts as a flag that is set true if any of the bump sensors are hit
    leftGo: a share that acts aa a boolean flag to allow the left motor to run when set high (1)
    rightGo: a share that acts as a boolean flag to allow the right motor to run when set high (1)
    '''
    def __init__(self, left_pins, right_pins,
                 sp_left, sp_right, leftGo, rightGo,
                 collision_mode):
        ''' Initializes a bump sensor object'''
        self.left_pins = left_pins
        self.right_pins = right_pins
        self.collision_mode = collision_mode

        self.sp_left = sp_left
        self.sp_right = sp_right
        self._leftGo = leftGo
        self._rightGo = rightGo

        self._state = S0_INIT

    def _left_hit(self):
        '''Returns True when the left bump sensors are hit and read low.'''
        return any(p.value() == 0 for p in self.left_pins)

    def _right_hit(self):
        '''Returns True when the right bump sensors are hit and read low.'''
        return any(p.value() == 0 for p in self.right_pins)

    def run(self):
        ''' runs one instance of polling the bump sensors'''
        while True:
            if self._state == S0_INIT:
                self._state = S1_WAIT

            elif self._state == S1_WAIT:
                if not (self._leftGo.get() and self._rightGo.get()):
                    pass

                elif self._left_hit() or self._right_hit():
                    self.collision_mode.put(1)

            yield self._state
