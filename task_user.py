''' This file demonstrates an example UI task using a custom class with a
    run method implemented as a generator
'''
from pyb import USB_VCP
from task_share import Share, Queue
import micropython


menu = (
"+------------------------------------------------------------------------------+\r\n"
"| ME 405 Romi Tuning Interface Help Menu                                       |\r\n"
"+---+--------------------------------------------------------------------------+\r\n"
"| h | Print help menu                                                          |\r\n"
"| k | Enter new motor gain values, sensor gain = 3500                          |\r\n"
"| s | CANNOT Choose a new setpoint!! Setpoint = 1200 ticks/s                   |\r\n"
"| c | Calibrate line sensors                                                   |\r\n"
"| g | Trigger step response and print results                                  |\r\n"
"+---+--------------------------------------------------------------------------+\r\n"
)


S0_INIT = micropython.const(0) # State 0 - initialiation
S1_CMD  = micropython.const(1) # State 1 - wait for character input
S2_COL  = micropython.const(2) # State 2 - wait for data collection to end
S3_DIS  = micropython.const(3) # State 3 - display the collected data
S4_GAIN  = micropython.const(4) # State 4 - gain values
S5_digit  = micropython.const(5) # State 5 - check digit
S6_Calibration = micropython.const(6) # state 6 - calibrate the sensors
UI_prompt = ">: "


class task_user:
    '''
    A class that represents a UI task. The task is responsible for reading user
    input over a serial port, parsing the input for single-character commands,
    and then manipulating shared variables to communicate with other tasks based
    on the user commands.
    '''

    # def __init__(self, leftMotorGo, rightMotorGo,share_kp, share_ki, share_setpoint, leftDataValues, leftTimeValues,
    #              rightDataValues, rightTimeValues, centroidData, centroidTime, statePredTime, statePredX, statePredY,
    #              statePredSL, stateMeasXL):
    def __init__(self, leftMotorGo, rightMotorGo,share_kp, share_ki, share_setpoint, leftDataValues, leftTimeValues,
                 rightDataValues, rightTimeValues, centroidData, centroidTime, statePredX, statePredY, sL_yhat, sL_meas, statetime,
                 share_calL, share_calD):
        '''
        Initializes a UI task object
        
        Args:
            leftMotorGo (Share):  A share object representing a boolean flag to
                                  start data collection on the left motor
            rightMotorGo (Share): A share object representing a boolean flag to
                                  start data collection on the right motor
            dataValues (Queue):   A queue object used to store collected encoder
                                  position values
            timeValues (Queue):   A queue object used to store the time stamps
                                  associated with the collected encoder data
        '''
        self._sensors = sensors                 # Will replaced with sensor reading queue
        self._state: int          = S0_INIT      # The present state
        
        self._leftMotorGo: Share  = leftMotorGo  # The "go" flag to start data
                                                 # collection from the left
                                                 # motor and encoder pair
        
        self._rightMotorGo: Share = rightMotorGo # The "go" flag to start data
                                                 # collection from the right
                                                 # motor and encoder pair
        self._share_kp: Share = share_kp
        self._share_ki: Share = share_ki
        self._calL: Share = share_calL          #calibration flag for light
        self._calD: Share = share_calD          #calibration flag for dark

        self._share_setpoint: Share = share_setpoint
        
        #self._ser: stream         = USB_VCP()    # A serial port object used to
        self._ser = USB_VCP()                                         # read character entry and to
                                                 # print output
        self._leftDataValues:  Queue   = leftDataValues
        self._leftTimeValues:  Queue   = leftTimeValues 
        self._rightDataValues: Queue   = rightDataValues
        self._rightTimeValues: Queue   = rightTimeValues
        self._centroidData:    Queue   = centroidData 
        self._centroidTime:    Queue   = centroidTime
        # self._statePredTime = statePredTime
        self._statePredX = statePredX
        self._statePredY = statePredY
        self._sL_yhat = sL_yhat
        self._sL_meas = sL_meas
        self._statetime = statetime
        # self._statePredSL = statePredSL
        # self._stateMeasXL = stateMeasXL                                      
                                                 
       ##### new adds 
        self._ser.write(b"User Task object instantiated\r\n")
        self._out_share = None


        self.digits:   set(str) = set(map(str,range(10)))  
        self._char_buf: str      = ""
        self._term:     set(str) = {"\r", "\n"}
        self._done = False
        self._kp: str      = ""
        self._kI: str      = ""
        self._kd: str      = ""
        self._kp_new = False
        self._ki_new = False
        self._valid_dig = False
        
    def run(self):
        '''
        Runs one iteration of the task
        '''
        
        while True:
            
            if self._state == S0_INIT: 
                self._ser.write(b"Initializing user task\r\n")
                self._ser.write(menu.encode())
                self._ser.write(UI_prompt.encode()) 
                self._state = S1_CMD
                
            elif self._state == S1_CMD: # Wait for UI commands
                if self._ser.any():
                    inChar = self._ser.read(1).decode()
                    if inChar in {"h","H"}:
                        self._ser.write(f" Now that you've hit {inChar} let's see how we can help\r\n")
                        self._ser.write(menu.encode())
                        self._state = S1_CMD
                    elif inChar in {"k","K"}:
                        self._ser.write(f"ooo {inChar} was hit for a new gain\r\n")
                        self._ser.write("What type of k value would you like (i or p): \r\n")
                        self._state = S4_GAIN
                    elif inChar in {"s","S"}:
                        self._ser.write(f" coolio {inChar} was hit\r\n")
                        self._out_share = self._share_setpoint
                        self._char_buf = ""
                        self._ser.write(f"What setpoint value would you like?\r\n")
                        self._state = S5_digit
                    elif inChar in {"g","G"}:   
                        self._ser.write(f" Fantastic! {inChar} was hit. Time to go :)\r\n\n") 
                        self._leftMotorGo.put(True)
                        self._rightMotorGo.put(True)
                        self._ser.write("Starting right motor loop...\r\n")
                        self._ser.write("Starting left motor loop...\r\n")
                        self._ser.write("Starting data collection...\r\n")
                        self._ser.write("Please wait... \r\n\n")
                        self._state = S2_COL
                    elif inChar in {"c","C"}:
                        self._ser.write("Calibrate for Dark(d) or Light(l)\r\n")
                        self._ser.write(UI_prompt)
                        self._state=S6_Calibration 
                
            elif self._state == S2_COL:
                # While the data is collecting (in the motor task) block out the
                # UI and discard any character entry so that commands don't
                # queue up in the serial buffer
                if self._ser.any(): self._ser.read(1)
                
                # When both go flags are clear, the data collection must have
                # ended and it is time to print the collected data.
                if not self._leftMotorGo.get() and not self._rightMotorGo.get():
                    self._ser.write("Step response complete...\r\n")
                    self._ser.write("Printing data...\r\n\n")
                    self._ser.write("--------------------\r\n")
                    self._ser.write(f"Setpoint: {self._share_setpoint.get()} mm/s\r\n")
                    self._ser.write(f"Kp:       {self._share_kp.get()} %*s/mm\r\n")
                    self._ser.write(f"Ki:       {self._share_ki.get()} %/mm\r\n")
                    self._ser.write("--------------------\r\n\n")
                    #self._ser.write("Time, Position\r\n")
                    self._ser.write("Time, Velocity\r\n")
                    self._state = S3_DIS
            
            elif self._state == S3_DIS:

                # ---- Print Left Motor Data ----
                if self._leftDataValues.any():
                    self._ser.write("Left Motor Data\r\n")
                    self._ser.write("Time, Velocity\r\n")
                    while self._leftDataValues.any():
                        #c_left = self._centroidData.get() if self._centroidData.any() else 0.0
                        # self._ser.write(
                        #     f"{self._leftTimeValues.get()},{self._leftDataValues.get()},{c_left}\r\n"
                        # )
                        self._ser.write(
                            f"{self._leftTimeValues.get()},{self._leftDataValues.get()}\r\n"
                        )
                    self._ser.write("--------------------\r\n\n")

                # ---- Print Right Motor Data ----
                if self._rightDataValues.any():
                    self._ser.write("Right Motor Data\r\n")
                    self._ser.write("Time, Velocity\r\n")
                    while self._rightDataValues.any():
                        self._ser.write(
                            f"{self._rightTimeValues.get()},{self._rightDataValues.get()}\r\n"
                        )
                    self._ser.write("--------------------\r\n\n")
                
                # ---- Print Centroid Data ----
                if self._centroidData.any():
                    self._ser.write("Centroid Data\r\n")
                    self._ser.write("Time, Normalized Centroid\r\n")
                    while self._centroidData.any():
                        #c_left = self._centroidData.get() if self._centroidData.any() else 0.0
                        self._ser.write(
                            f"{self._centroidTime.get()},{self._centroidData.get()}\r\n"
                        )
                # ---- Print X and Y Data ----
                if self._statePredX.any():
                     self._ser.write("State Prediction Data \r\n")
                #     self._ser.write("Time, X, Y, xL, sL\r\n")
                     self._ser.write("X, Y\r\n")
                     while self._statePredX.any():
                #         self._ser.write(f"{self._statePredTime.get()}, {self._statePredX.get()}, {self._statePredY.get()}, {self._stateMeasXL.get()}, {self._statePredSL.get()}\r\n")
                         self._ser.write(f"{self._statePredX.get()}, {self._statePredY.get()}\r\n")

                # ---- Print sL Data ----
                if self._sL_yhat.any():
                     self._ser.write("State Prediction and Measured Data \r\n")
                #     self._ser.write("Time, X, Y, xL, sL\r\n")
                     self._ser.write("time, sL Estimated, sL Measured\r\n")
                     while self._sL_yhat.any() and self._sL_meas.any() and self._statetime.any():
                     # while self._sL_yhat.any():
                         self._ser.write(f"{self._statetime.get()}, {self._sL_yhat.get()}, {self._sL_meas.get()}\r\n")

                self._ser.write("END\r\n")
                self._ser.write("--------------------\r\n")

                # After printing both, go back to command state
                self._ser.write(menu.encode())
                self._ser.write(UI_prompt.encode())
                self._state = S1_CMD
            elif self._state == S4_GAIN:
                # reset each time
                self._kp_new = False
                self._ki_new = False

                if self._ser.any():
                    k_choose = self._ser.read(1).decode()

                    if k_choose in {"p", "P"}:
                        self._out_share = self._share_kp
                        self._kp_new = True
                    elif k_choose in {"i", "I"}:
                        self._out_share = self._share_ki
                        self._ki_new = True
                    else:
                        self._ser.write("Pick p or i\r\n")
                        yield self._state
                        continue

                    self._char_buf = ""
                    self._ser.write(f"What k{k_choose} value would you like?\r\n")
                    self._state = S5_digit


            elif self._state == S5_digit:
                if self._ser.any():
                    ch = self._ser.read(1).decode()

                    if ch in self.digits:
                        self._ser.write(ch)
                        self._char_buf += ch

                    elif ch == "." and "." not in self._char_buf:
                        self._ser.write(ch)
                        self._char_buf += ch

                    elif ch == "-" and len(self._char_buf) == 0:
                        self._ser.write(ch)
                        self._char_buf += ch


                    # treat either DEL (0x7f) or BS (0x08) as backspace (some terminals send one or the other)
                    if ch in ("\x7f", "\x08"):
                        if len(self._char_buf) > 0:
                            # remove last char from buffer
                            self._char_buf = self._char_buf[:-1]
                            # echo a backspace sequence to the terminal so the user sees the deletion
                            # \x08 is backspace; some terminals need "\x08 \x08" to erase the character
                            try:
                                self._ser.write(b"\x08 \x08")
                            except TypeError:
                                # if .write() expects str on this build, fall back to encoded str
                                self._ser.write("\x08 \x08".encode())
                        # else: nothing to delete, silently ignore

                    elif ch in self._term:
                        if len(self._char_buf) == 0:
                            self._ser.write("\r\nValue not changed\r\n")
                        elif self._char_buf not in {"-", "."}:
                            val = float(self._char_buf)
                            self._out_share.put(val)
                            self._ser.write("\r\nValue set to {}\r\n".format(val))

                        self._char_buf = ""
                        #self._state = S1_CMD
                        self._state = S0_INIT
                        self._ser.write(UI_prompt.encode())
            elif self._state == S6_Calibration:
                if self._ser.any():
                    inChar=self._ser.read(1).decode()
                    if inChar in {"d","D"}:
                        self._calD.put(True)
                        self._ser.write(f"{inChar}\r\n")
                        self._ser.write("Dark calibration complete\r\n")
                        self._ser.write(UI_prompt)
                    elif inChar in {"l","L"}:
                        self._calL.put(True)
                        self._ser.write(f"{inChar}\r\n")
                        self._ser.write("Light calibration complete\r\n")
                        self._ser.write(UI_prompt)
                    self._state = S1_CMD

    
            yield self._state

