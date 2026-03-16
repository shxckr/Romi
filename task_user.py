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
"| s | CANNOT Choose a new setpoint!! Setpoint = 175 mm/s                       |\r\n"
"| c | Calibrate line sensors                                                   |\r\n"
"| g | Trigger step response and print results                                  |\r\n"
"| o | Select Optimized Gains                                                   |\r\n"
"| l | Select line follow Gains                                                 |\r\n"
"| q | Stop Romi                                                                |\r\n"
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
    def __init__(self, leftMotorGo, rightMotorGo,share_kp, share_ki, leftDataValues, leftTimeValues,
                 rightDataValues, rightTimeValues, centroidData, centroidTime, statePredX, statePredY, sL_yhat, sL_meas, statetime,
                 share_calL, share_calD, db_share, share_lineKp, share_lineKi, sensors):
        '''
        Initializes a UI task object
        Args:
            leftMotorGo (Share):    A share object representing a boolean flag to
                                    start data collection on the left motor
            rightMotorGo (Share):   A share object representing a boolean flag to
                                    start data collection on the right motor
            shareKp (Share):        A share object to store Kp value for motor control
            shareKi (Share):        A share object to store Ki value for motor control
            leftDataValues(Queue):  A queue object to share the left motor data values
            leftTimeValues(Queue):  A queue object to share the left motor time values
            rightDataValues(Queue): A queue object to share the right motor data values
            rightTimeValues(Queue): A queue object to share the right motor time values
            centroidData(Queue):    A queue object to share the line centroid data values
            centroidTime(Queue):    A queue object to share the line centroid time values
            statePredX(Queue):      A queue object to share the X state prediction values
            statePredY(Queue):      A queue object to share the Y state prediction values
            sL_yhat(Queue):         A queue object to share the sL state prediction values
            sL_meas(Queue):         A queue object to share the sL measured values
            statetime(Queue):       A queue object to share the state time values
            share_calL(Share):      A share object storing a boolean which determines 
                                    when calibration for light
            share_calD(Share):      A share object storing a boolean which determines 
                                    when calibration for dark
            db_share(Share):        A share object for debugging purposes
            share_lineKp(Share):    A share object to store Kp value for line follow control
            share_lineKi(Share):    A share object to store Ki value for line follow control
        '''
        self._state: int          = S0_INIT                 
        self._leftMotorGo: Share  = leftMotorGo                                                                         
        self._rightMotorGo: Share = rightMotorGo            
        self._share_kp: Share = share_kp                    
        self._share_ki: Share = share_ki                    
        self._calL: Share = share_calL                      
        self._calD: Share = share_calD                                           
        self._leftDataValues:  Queue   = leftDataValues     
        self._leftTimeValues:  Queue   = leftTimeValues     
        self._rightDataValues: Queue   = rightDataValues    
        self._rightTimeValues: Queue   = rightTimeValues
        self._centroidData:    Queue   = centroidData 
        self._centroidTime:    Queue   = centroidTime
        self._statePredX = statePredX
        self._statePredY = statePredY
        self._sL_yhat = sL_yhat
        self._sL_meas = sL_meas
        self._statetime = statetime                                      
        self._share_lineKp = share_lineKp
        self._share_lineKi = share_lineKi            
        self._out_share = None
        self.db_share = db_share
        self._sensors = sensors
        self.lineFollowGain = False # flag whether line follow gains are set or motor gains
        self.digits:   set(str) = set(map(str,range(10)))  
        self._char_buf: str      = ""
        self._term:     set(str) = {"\r", "\n"}
        self._ser = USB_VCP()                               # serial object for usb communication
        self._ser.write(b"User Task object instantiated\r\n")
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
                    if inChar in {"h","H"}: # Display command menu
                        self._ser.write(f" Now that you've hit {inChar} let's see how we can help\r\n")
                        self._ser.write(menu.encode())
                        self._state = S1_CMD
                    elif inChar in {"k","K"}: # set motor gains
                        self._ser.write(f"ooo {inChar} was hit for a new gain\r\n")
                        self._ser.write("What type of k value would you like (i or p): \r\n")
                        self._ser.write(UI_prompt)
                        self.lineFollowGain = False
                        self._state = S4_GAIN
                    elif inChar in {"s","S"}: # set motor setpoint
                        self._ser.write(f" coolio {inChar} was hit\r\n")
                        self._out_share = self._share_setpoint
                        self._char_buf = ""
                        self._ser.write(f"What setpoint value would you like?\r\n")
                        self._ser.write(UI_prompt)
                        self._state = S5_digit
                    elif inChar in {"g","G"}: # run motors
                        self._ser.write(f" Fantastic! {inChar} was hit. Time to go :)\r\n\n") 
                        self._leftMotorGo.put(True)
                        self._rightMotorGo.put(True)
                        self._ser.write("Starting right motor loop...\r\n")
                        self._ser.write("Starting left motor loop...\r\n")
                        self._ser.write("Starting data collection...\r\n")
                        self._ser.write("Please wait... \r\n\n")
                        self._state = S2_COL
                    elif inChar in {"c","C"}: # calibrate sensors
                        self._ser.write(f"{inChar} was hit for calibration\r\n")
                        self._ser.write("Calibrate for Dark(d) or Light(l)\r\n")
                        self._ser.write(UI_prompt)
                        self._state=S6_Calibration 
                    elif inChar in {'o','O'}: # load optimized motor gains
                        self._share_kp.put(0.04)
                        self._share_ki.put(0.01)
                        self._ser.write("Optimized gains have been set\r\n")
                        self._ser.write(UI_prompt)
                        self._state = S1_CMD
                    elif inChar in {"l","L"}: # set line follow gains
                        self._ser.write(f"ooo {inChar} was hit for a line follow gain\r\n")
                        self._ser.write("What type of k value would you like (i or p): \r\n")
                        self._ser.write(UI_prompt)
                        self.lineFollowGain = True
                        self._state = S4_GAIN
            elif self._state == S2_COL:
                # While the data is collecting (in the motor task) check the
                # characters for a quit command to stop data collection
                self._ser.write(f"lGO:{str(self._leftMotorGo.get())} rGO:{str(self._rightMotorGo.get())}\r\n")
                if self._ser.any():
                    inChar = self._ser.read(1).decode()
                    if inChar in {"q","Q"}:
                        self._ser.write("Stopping Motors\r\n")
                        self._leftMotorGo.put(False)
                        self._rightMotorGo.put(False)
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
                    self._ser.write("Time, Velocity\r\n")
                    self._state = S3_DIS
            
            elif self._state == S3_DIS:    
                # ---- Print Left Motor Data ----
                if self._leftDataValues.any():
                    self._ser.write("Left Motor Data\r\n")
                    self._ser.write("Time, Velocity\r\n")
                    while self._leftDataValues.any():
                        self._ser.write(f"{self._leftTimeValues.get()},{self._leftDataValues.get()}\r\n")
                    self._ser.write("--------------------\r\n\n")
                # ---- Print Right Motor Data ----
                if self._rightDataValues.any():
                    self._ser.write("Right Motor Data\r\n")
                    self._ser.write("Time, Velocity\r\n")
                    while self._rightDataValues.any():
                        self._ser.write(f"{self._rightTimeValues.get()},{self._rightDataValues.get()}\r\n")
                    self._ser.write("--------------------\r\n\n")
                # ---- Print Centroid Data ----
                if self._centroidData.any():
                    self._ser.write("Centroid Data\r\n")
                    self._ser.write("Time, Normalized Centroid\r\n")
                    while self._centroidData.any():
                        self._ser.write(f"{self._centroidTime.get()},{self._centroidData.get()}\r\n")
                # ---- Print X and Y Data ----
                if self._statePredX.any():
                     self._ser.write("State Prediction Data \r\n")
                     self._ser.write("X, Y\r\n")
                     while self._statePredX.any():
                         self._ser.write(f"{self._statePredX.get()}, {self._statePredY.get()}\r\n")
                # ---- Print sL Data ----
                if self._sL_yhat.any():
                     self._ser.write("State Prediction and Measured Data \r\n")
                     self._ser.write("time, sL Estimated, sL Measured\r\n")
                     while self._sL_yhat.any() and self._sL_meas.any() and self._statetime.any():
                         self._ser.write(f"{self._statetime.get()}, {self._sL_yhat.get()}, {self._sL_meas.get()}\r\n")
                self._ser.write("END\r\n")
                self._ser.write("--------------------\r\n")

                # After printing both, go back to command state
                self._ser.write(menu.encode())
                self._ser.write(UI_prompt.encode())
                self._state = S1_CMD
            elif self._state == S4_GAIN:
                # check characters for valid imputs
                if self._ser.any():
                    k_choose = self._ser.read(1).decode()
                    if k_choose in {"p", "P"}:
                        self._ser.write(f"{k_choose})\r\n")
                        # determine which propotional gain, line follow or motor, to update
                        if self.lineFollowGain: self._out_share = self._share_lineKp
                        else: self._out_share = self._share_kp
                    elif k_choose in {"i", "I"}:
                        self._ser.write(f"{k_choose})\r\n")
                        # determine which integral gain, line follow or motor, to update
                        if self.lineFollowGain: self._out_share = self._share_lineKi
                        else: self._out_share = self._share_ki
                    else:
                        # if invalid character, prompt again
                        self._ser.write("Pick p or i\r\n")
                        yield self._state
                        continue
                    self._char_buf = ""
                    self._ser.write(f"What k{k_choose} value would you like?\r\n")
                    self._state = S5_digit
            elif self._state == S5_digit:
                # check buffer for valid characters
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
                    if ch in ("\x7f", "\x08"):
                        # if delete or backspace is hit, check if buffer has characters
                        if len(self._char_buf) > 0:
                            # remove last char from buffer
                            self._char_buf = self._char_buf[:-1]
                            # echo a backspace sequence to the terminal so the user sees the deletion
                            try:
                                self._ser.write(b"\x08 \x08")
                            except TypeError:
                                self._ser.write("\x08 \x08".encode())
                    elif ch in self._term:
                        if len(self._char_buf) == 0:
                            self._ser.write("\r\nValue not changed\r\n")
                        elif self._char_buf not in {"-", "."}:
                            val = float(self._char_buf)
                            self._out_share.put(val)
                            self._ser.write("\r\nValue set to {}\r\n".format(val))
                        self._char_buf = ""
                        self._state = S1_CMD
                        self._ser.write(UI_prompt.encode())
            elif self._state == S6_Calibration:
                if self._ser.any():
                    inChar=self._ser.read(1).decode()
                    if inChar in {"d","D"}:
                        self._calD.put(True)
                        self._ser.write(f"{inChar}\r\n")
                        self._ser.write(str(self._sensors.getDark()))
                        self._ser.write("Dark calibration complete\r\n")
                        self._ser.write(UI_prompt)
                        self._state = S1_CMD
                    elif inChar in {"l","L"}:
                        self._calL.put(True)
                        self._ser.write(f"{inChar}\r\n")
                        self._ser.write(str(self._sensors.getLight()))
                        self._ser.write("Light calibration complete\r\n")
                        self._ser.write(UI_prompt)
                        self._state = S1_CMD
            yield self._state

