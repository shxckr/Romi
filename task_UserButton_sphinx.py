'''
File to detect when the user button is pressed and set a flag in a share to be read by other tasks
'''
try:
    from pyb import Pin
except Exception:
    class Pin:
        def value(self): return 1

try:
    import micropython
except Exception:
    class micropython:
        @staticmethod
        def const(x): return x

try:
    from task_share import Share
except Exception:
    class Share:
        def put(self, *a): pass
        def get(self): return False

S0_INIT = micropython.const(0) # Initialization state
S1_Poll = micropython.const(1) # Polling state
S2_Run = micropython.const(2) # Raise flag state

class task_UButton:
    def __init__(self, pin: Pin, flag_share: Share):
        self._button = pin
        self._flag_share = flag_share
        self._state=S0_INIT
        self._flag_share.put(False)

    def update(self):
        return not self._button.value() == True # active low button, returns True when pressed
    def run(self):
        while True:
            if self._state == S0_INIT:
                self._state = S1_Poll
            elif self._state == S1_Poll:
                if self.update():
                    self._flag_share.put(True) # Set flag when button is pressed
                    self._state = S2_Run
            elif self._state == S2_Run:
                if not self._flag_share.get(): # wait for flag to be acknowledged 
                    self._state = S1_Poll
            yield self._state
