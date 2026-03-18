import micropython
from pyb import millis 
from linesensors import LineSensors
from HeadingHoldController import HeadingHoldController
from gc import collect

S0_INIT            = micropython.const(0)
S1_LINE            = micropython.const(1)
S2_EnterGarage            = micropython.const(2)
S3_GARAGETURN      = micropython.const(3)
S4_GARAGESTRAIGHT = micropython.const(4)
S5_BACKUP            = micropython.const(5)
S6_OUTTURN            = micropython.const(6)
S7_CHECK2                = micropython.const(7)
S8_TURN                = micropython.const(8)
S9_WIGGLE                = micropython.const(9)
S10_HALFCIRCLE                = micropython.const(10)
S11_HALFTURN                = micropython.const(11)
S12_TOSTART                = micropython.const(12)
S13_Straight                = micropython.const(13)
S14_END                = micropython.const(14)

class task_course:

    def __init__(self, sL_share, sR_share, collision_mode, yolo_mode,
             sp_left, sp_right, leftGo, rightGo, line_sensor,
             imu_heading_share, imu_yaw_share, ser, initHeadSh, shUB_flag, baseSh):
         self.sL_share = sL_share
         self.sR_share = sR_share
         self.collision_mode = collision_mode
         self.yolo_mode = yolo_mode
         self.sp_left = sp_left
         self.sp_right = sp_right
         self._leftGo = leftGo
         self._rightGo = rightGo
         self.line_sensor = line_sensor
         self._sL_start = 0
         self._sR_start = 0
         self._state = S0_INIT
         self._t0 = 0
         self.imu_heading_share = imu_heading_share
         self.imu_yaw_share = imu_yaw_share
         self._initHeadSh = initHeadSh
         self.UB_flag = shUB_flag
         self.baseSh = baseSh
         self.heading_hold = HeadingHoldController(self.imu_heading_share, self._initHeadSh,
                                          target_heading=180,
                                          base_speed=133.0,
                                          kp=4,ki=0.2, #3.8
                                          min_speed=60.0,
                                          max_speed=400.0)
         self.heading_hold_ninety = HeadingHoldController(self.imu_heading_share, self._initHeadSh,
                                          target_heading=90.0,
                                          base_speed=183.0,
                                          kp=4,ki=0.2,
                                          min_speed=60.0,
                                          max_speed=400.0)
         self.heading_hold_5 = HeadingHoldController(self.imu_heading_share, self._initHeadSh,
                                          target_heading=90.0,
                                          base_speed=-183.0,
                                          kp=4,ki=0.2,
                                          min_speed=60.0,
                                          max_speed=400.0)
         self.heading_hold_6 = HeadingHoldController(self.imu_heading_share, self._initHeadSh,
                                          target_heading=90.0, # change back to 90 when full testing
                                          base_speed=183, #182*2?
                                          kp=4,ki=0.2,
                                          min_speed=60.0,
                                          max_speed=400.0)
         self.heading_hold_13 = HeadingHoldController(self.imu_heading_share, self._initHeadSh,
                                          target_heading=270.0, # change back to 90 when full testing
                                          base_speed=183, #182*2?
                                          kp=4,ki=0.2,
                                          min_speed=60.0,
                                          max_speed=400.0)
         self._ser = ser
    

    def run(self):
        while True:
            if self._state == S0_INIT:
                self.collision_mode.put(0)
                self.yolo_mode.put(0)
                if self.UB_flag.get() == 1:
                    self._leftGo.put(True)
                    self._rightGo.put(True)
                    self._sL_start = self.sL_share.get()
                    self._heading0 = self.imu_heading_share.get()
                    self._initHeadSh.put(self._heading0)
                    self._ser.write("State 0 Complete"+"\r\n")
                    self._sR_start = self.sR_share.get()
                    self.UB_flag.put(0)
                    self._state = S1_LINE
                    
            elif self._state == S1_LINE:
                    self.yolo_mode.put(0)
                    #norm_vals = self.line_sensor.read_normalized()
                    deltaL = abs(self.sL_share.get() - self._sL_start)
                    self._ser.write("DeltaL: "+str(deltaL)+"\r\n")

                    if deltaL >= 1735:
                        self.yolo_mode.put(1)
                        self._t0 = millis()
                        self._sL_start = self.sL_share.get()
                        
                        self._ser.write("State 1 Complete"+"\r\n")
                        self._state = S2_EnterGarage

            elif self._state == S2_EnterGarage:
                        deltaL = abs(self.sL_share.get() - self._sL_start)
                        left_cmd, right_cmd, heading_error = self.heading_hold_ninety.get_wheel_speeds()
                        self.sp_left.put(left_cmd)
                        self.sp_right.put(right_cmd)
                        self._ser.write("Heading: "+str(self.imu_heading_share.get())+"\r\n")
                        self._leftGo.put(1)
                        self._rightGo.put(1)
                        if deltaL >= 165:
                            self._leftGo.put(0)
                            self._rightGo.put(0)
                            self._sL_start = self.sL_share.get()
                            self._sR_start = self.sR_share.get()
                            self._ser.write("State 2 Complete"+"\r\n")
                            self._state= S3_GARAGETURN
            elif self._state == S3_GARAGETURN:
                deltaR = abs(self.sR_share.get() - self._sR_start)
                self.sp_left.put(280)
                self.sp_right.put(-280)
                self._leftGo.put(1)
                self._rightGo.put(1)
                # heading now is between 0 and 360
                self._headingCur = (self.imu_heading_share.get() - self._heading0)%360
                #self._ser.write("Heading: "+str(self._headingCur)+"\r\n")
                
                if deltaR >= 4 and self._headingCur >= 160:
                    self._leftGo.put(0)
                    self._rightGo.put(0)
                    self._sR_start = self.sR_share.get()
                    self._sL_start = self.sL_share.get()
                    self._t0 = millis()
                    self._ser.write("State 3 Complete"+"\r\n")
                    self._state = S4_GARAGESTRAIGHT # only testing state 3

            elif self._state == S4_GARAGESTRAIGHT:
                '''This shouldn't be calculating before we start moving because it accumulates error
                    but it works to counter the fact that we're overshooting 180 and close to a pillar'''
                left_cmd, right_cmd, heading_error = self.heading_hold.get_wheel_speeds()

                # if we just entered this state we expect self._t0 to be set when leaving previous state
                # use the controller outputs to drive straight toward 180 deg
                    # normal forward with heading hold
                self.sp_left.put(left_cmd)
                self.sp_right.put(right_cmd)
                self._leftGo.put(1)
                self._rightGo.put(1)
                self._headingCur = (self.imu_heading_share.get() - self._heading0)%360
                self._ser.write("Heading: "+str(self._headingCur)+"\r\n")                
                if self.collision_mode.get() == 1:
                #if deltaL >= 100:
                    self.sp_left.put(0)
                    self.sp_right.put(0)
                    self._leftGo.put(0)
                    self._rightGo.put(0)
                    self._sL_start = self.sL_share.get()
                    self._headingCur = (self.imu_heading_share.get() - self._heading0)%360
                    self.collision_mode.put(0)
                    # new idea
                    #self._ser.write("Collision Detected! Heading: "+str(self._headingCur)+"\r\n")
                    self._ser.write("State 4 Complete\r\n")
                    self._state = S5_BACKUP
            elif self._state == S5_BACKUP:

                deltaL = abs(self.sL_share.get() - self._sL_start)

                left_cmd, right_cmd, heading_error = self.heading_hold_5.get_wheel_speeds()

                self.sp_left.put(left_cmd)
                self.sp_right.put(right_cmd)

                self._leftGo.put(1)
                self._rightGo.put(1)
                self._headingCur = (self.imu_heading_share.get() - self._heading0)%360
                #self._ser.write("Heading: "+str(self._headingCur)+"\r\n")
                if deltaL >= 100:
                    self._leftGo.put(0)
                    self._rightGo.put(0)
                    #self._sL_start = self.sL_share.get()
                    self._headingCur = (self.imu_heading_share.get() - self._heading0)%360
                    self._sR_start = self.sR_share.get()
                    #self._headingCur = (self.imu_heading_share.get() - self._heading0)%360
                    #self._ser.write("Heading: "+str(self._headingCur)+"\r\n")
                    self._ser.write("State 5 Completed\r\n")
                    self._state = S6_OUTTURN

            elif self._state == S6_OUTTURN:
                deltaR = abs(self.sR_share.get() - self._sR_start)
                self.sp_left.put(-360)
                self.sp_right.put(360)
                self._leftGo.put(1)
                self._rightGo.put(1)
                #self._headingCur = (self.imu_heading_share.get() - self._heading0)%360
                #self._ser.write("Heading: "+str(self._headingCur)+"\r\n")
                
                #if self._headingCur <= 90:
                if deltaR >= 5:
                    self._rightGo.put(0)
                    self._leftGo.put(0)
                    self._sR_start = self.sR_share.get()
                    self._sL_start = self.sL_share.get()
                    self.collision_mode.put(0)
                    self._ser.write("State 6 Complete\r\n")
                    #self._ser.write("Yolo: "+str(self.yolo_mode.get())+"\r\n")
                    self._ser.write("Collision: "+str(self.collision_mode.get())+"\r\n")
                    self._state = S7_CHECK2

            elif self._state == S7_CHECK2:
                    spl, spr, errr = self.heading_hold_6.get_wheel_speeds()
                    self.sp_left.put(spl)
                    self.sp_right.put(spr)
                    self._leftGo.put(1)
                    self._rightGo.put(1)
                    self._headingCur = (self.imu_heading_share.get()-self._heading0)%360
                    #self._ser.write("Heading: "+str(self._headingCur)+"\r\n")
                    deltaL = abs(self.sL_share.get() - self._sL_start)
                    #read norm vals and detect when centroid is only in the right
                    if deltaL >= 300 :
                        self._t0 = millis()
                        #self._ser.write("State 7 Complete"+"\r\n")
                        self._leftGo.put(0)
                        self._rightGo.put(0)
                        #self._sL_start = self.sL_share.get()
                        #self._sR_start = self.sR_share.get()
                        self._state = S8_TURN
                        
            elif self._state == S8_TURN:
                self.yolo_mode.put(0)
                norm_vals = self.line_sensor.read_normalized()
                deltaR = abs(self.sR_share.get() - self._sR_start)
                self.sp_left.put(350)
                self.sp_right.put(-350)
                self._leftGo.put(1)
                self._rightGo.put(1)

                #self._headingCur = (self.imu_heading_share.get() - self._heading0) % 360
                #self._ser.write("Heading: " + str(self._headingCur) + "\r\n")
                #self._headingCur >= 90

                #if  sum(val > 0.2 for val in norm_vals) >= (len(norm_vals) // 2 + 1):
                #if self.line_sensor.read_normalized()[3] >= 0.8 and self.line_sensor.read_normalized()[4] >= 0.8: 
                if deltaR >= 5:
                    self._leftGo.put(0)
                    self._rightGo.put(0)
                    #self._sR_start = self.sR_share.get()
                    self._sL_start = self.sL_share.get()
                    self._ser.write("State 8 Complete\r\n")
                    collect()
                    self._sL_start = self.sL_share.get()
                    self._state = S9_WIGGLE

            elif self._state == S9_WIGGLE:
                # sL = self.sL_share.get()
                self.yolo_mode.put(0)
                self.baseSh.put(115) #
                self._leftGo.put(1)
                self._rightGo.put(1)
                self.yolo_mode.put(0)
    
                # norm_vals = self.line_sensor.read_normalized()
                deltaL = abs(self.sL_share.get() - self._sL_start)
                #self._ser.write(f"deltaL: {deltaL}\r\n")
                if deltaL >= 990:
                    collect()
                    #self._leftGo.put(0)
                    #self._rightGo.put(0)
                    #self.yolo_mode.put(1)
                    #self._t0 = millis()
                    self._sL_start = self.sL_share.get()
                    self._ser.write("State 9 Complete"+"\r\n")
                    #self._sL_start = self.sL_share.get()
                    self._state = S10_HALFCIRCLE
                    
            elif self._state == S10_HALFCIRCLE:
                deltaL = abs(self.sL_share.get() - self._sL_start)
                if deltaL >= 1200:
                    self._leftGo.put(0)
                    self._rightGo.put(0)
                    self.yolo_mode.put(1)
                    s11 = 1
                    self._state = S11_HALFTURN
                    collect()
            elif self._state == S11_HALFTURN:
                self._headingCur = (self.imu_heading_share.get() - self._heading0) % 360
                if s11 == 1:
                    self.sp_left.put(-300)
                    self.sp_right.put(300)
                    self._leftGo.put(1)
                    self._rightGo.put(1)
                    s11 = 0
                    
                elif 180 <= self._headingCur <= 200:
                    self._leftGo.put(0)
                    self._rightGo.put(0)
                    self._sL_start = self.sL_share.get()
                    #self._sL_start = self.sL_share.get()
                    #self._ser.write("State 11 Complete\r\n")
                    self.yolo_mode.put(0)
                    self.baseSh.put(175)
                    self._state = S12_TOSTART
            elif self._state == S12_TOSTART:
                self._leftGo.put(1)
                self._rightGo.put(1)
                deltaL = abs(self.sL_share.get() - self._sL_start)
                if deltaL >= 550:
                    self._leftGo.put(0)
                    self._rightGo.put(0)
                    self.yolo_mode.put(1)   
                    self._sL_start = self.sL_share.get()
                    self._state = S13_Straight
            elif self._state == S13_Straight:
                        deltaL = abs(self.sL_share.get() - self._sL_start)
                        left_cmd, right_cmd, heading_error = self.heading_hold_13.get_wheel_speeds()
                        self.sp_left.put(left_cmd)
                        self.sp_right.put(right_cmd)
                        self._ser.write("Heading: "+str(self.imu_heading_share.get())+"\r\n")
                        self._leftGo.put(1)
                        self._rightGo.put(1)
                        if deltaL >= 100:
                            self._leftGo.put(0)
                            self._rightGo.put(0)
                            self._sL_start = self.sL_share.get()
                            self._sR_start = self.sR_share.get()
                            self._ser.write("State 2 Complete"+"\r\n")
                            self._state= S14_END
                    
            elif self._state == S14_END:
                 self.sp_left.put(300)
                 self.sp_right.put(300)
                 self._leftGo.put(1)
                 self._rightGo.put(1)
                 self._headingCur = (self.imu_heading_share.get() - self._heading0) % 360
                 if self._headingCur >= 0:
                    self._leftGo.put(0)
                    self._rightGo.put(0)
                 
                    self._state = S0_INIT
            yield self._state