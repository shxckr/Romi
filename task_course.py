import micropython
import pyb
from linesensors import LineSensors
from HeadingHoldController import HeadingHoldController

S0_INIT            = micropython.const(0)
S1_LINE            = micropython.const(1)
S2_EnterGarage            = micropython.const(2)
S3_GARAGETURN      = micropython.const(3)
S4_GARAGESTRAIGHT = micropython.const(4)
S5_BACKUP            = micropython.const(5)
S6_OUTTURN            = micropython.const(6)
S7_NEXT                = micropython.const(7)


class task_course:

    def __init__(self, sL_share, sR_share, collision_mode, yolo_mode,
             sp_left, sp_right, leftGo, rightGo, line_sensor,
             imu_heading_share, imu_yaw_share, ser, initHeadSh):
         self.sL_share = sL_share
         self.sR_share = sR_share
         self.collision_mode = collision_mode
         self.yolo_mode = yolo_mode
         self.sp_left = sp_left
         self.sp_right = sp_right
         self.leftGo = leftGo
         self.rightGo = rightGo
         self.line_sensor = line_sensor
         self._sL_start = 0
         self._sR_start = 0
         self._state = S0_INIT
         self._t0 = 0
         self.imu_heading_share = imu_heading_share
         self.imu_yaw_share = imu_yaw_share
         self._initHeadSh = initHeadSh
         self.heading_hold = HeadingHoldController(self.imu_heading_share, self._initHeadSh,
                                          target_heading=155.0,
                                          base_speed=183.0,
                                          kp=4,
                                          min_speed=60.0,
                                          max_speed=400.0)
         self.heading_hold_ninety = HeadingHoldController(self.imu_heading_share, self._initHeadSh,
                                          target_heading=85.0,
                                          base_speed=183.0,
                                          kp=4,
                                          min_speed=60.0,
                                          max_speed=400.0)
         self.heading_hold_5 = HeadingHoldController(self.imu_heading_share, self._initHeadSh,
                                          target_heading=0.0,
                                          base_speed=-183.0,
                                          kp=2,
                                          min_speed=60.0,
                                          max_speed=400.0)
         self._ser = ser
    

    def run(self):
        while True:
            if self._state == S0_INIT:
                self.collision_mode.put(0)
                self.yolo_mode.put(0)
                self.sp_left.put(0)
                self.sp_right.put(0)
                if self.leftGo.get() and self.rightGo.get():
                    self._sL_start = self.sL_share.get()
                    self._heading0 = self.imu_heading_share.get()
                    self._initHeadSh.put(self._heading0)
                    self._state = S1_LINE
                    self._ser.write(str(self._state)+"STATE\r\n")

            elif self._state == S1_LINE:
                sL = self.sL_share.get()
                self.yolo_mode.put(0)
                norm_vals = self.line_sensor.read_normalized()
                deltaL = abs(self.sL_share.get() - self._sL_start)

                if deltaL >= 1735:
                    self.leftGo.put(0)
                    self.rightGo.put(0)
                    self.yolo_mode.put(1)
                    self._t0 = pyb.millis()
                    self._sL_start = self.sL_share.get()
                    self._ser.write(str(self._state)+"STATE\r\n")
                    self._state = S2_EnterGarage
            elif self._state == S2_EnterGarage:
                    if pyb.millis() - self._t0 >= 2000:
                        deltaL = abs(self.sL_share.get() - self._sL_start)
                        left_cmd, right_cmd, heading_error = self.heading_hold_ninety.get_wheel_speeds()
                        self.sp_left.put(left_cmd)
                        self.sp_right.put(right_cmd)
                        self.leftGo.put(1)
                        self.rightGo.put(1)
                        if deltaL >= 150:
                            self.leftGo.put(0)
                            self.rightGo.put(0)
                            self._sL_start = self.sL_share.get()
                            self._sR_start = self.sR_share.get()
                            self._state= S3_GARAGETURN
            elif self._state == S3_GARAGETURN:
                deltaR = abs(self.sR_share.get() - self._sR_start)
                self.sp_left.put(360)
                self.sp_right.put(-360)
                self.leftGo.put(1)
                self.rightGo.put(1)
                self._headingCur = self.imu_heading_share.get() - self._heading0
                self._ser.write("Heading: "+str(self._headingCur)+"\r\n")
                
                if deltaR >= 30 and self._headingCur >= 160:
                    self.leftGo.put(0)
                    self.rightGo.put(0)
                    self._sR_start = self.sR_share.get()
                    self._sL_start = self.sL_share.get()
                    self._t0 = pyb.millis()
                    self._state = S4_GARAGESTRAIGHT

            elif self._state == S4_GARAGESTRAIGHT:
                left_cmd, right_cmd, heading_error = self.heading_hold.get_wheel_speeds()

                # if we just entered this state we expect self._t0 to be set when leaving previous state
                # use the controller outputs to drive straight toward 180 deg
                if pyb.millis() - self._t0 >= 1000:
                    # normal forward with heading hold
                    self.sp_left.put(left_cmd)
                    self.sp_right.put(right_cmd)
                    self.leftGo.put(1)
                    self.rightGo.put(1)

                else:
                    self.sp_left.put(0)
                    self.sp_right.put(0)

                if self.collision_mode.get() == 1:
                #if deltaL >= 100:
                    self.sp_left.put(0)
                    self.sp_right.put(0)
                    self.leftGo.put(0)
                    self.rightGo.put(0)
                    self._sL_start = self.sL_share.get()
                    # new idea
                    self._heading4 = self.imu_heading_share.get()
                    self._initHeadSh.put(self._heading4)
                    self._state = S5_BACKUP
            elif self._state == S5_BACKUP:

                deltaL = abs(self.sL_share.get() - self._sL_start)

                left_cmd, right_cmd, heading_error = self.heading_hold_5.get_wheel_speeds()

                # reverse the commands
                self.sp_left.put(left_cmd)
                self.sp_right.put(right_cmd)

                self.leftGo.put(1)
                self.rightGo.put(1)

                if deltaL >= 100:
                    self.yolo_mode.put(0)
                    self.leftGo.put(0)
                    self.rightGo.put(0)
                    self._sL_start = self.sL_share.get()
                    self._sR_start = self.sR_share.get()
                    self._state = S6_OUTTURN

            elif self._state == S6_OUTTURN:
                deltaR = abs(self.sR_share.get() - self._sR_start)
                self.sp_left.put(-360)
                self.sp_right.put(360)
                self.leftGo.put(1)
                self.rightGo.put(1)
                self._headingCur = self.imu_heading_share.get() - self._heading0
                self._ser.write("Heading: "+str(self._headingCur)+"\r\n")
                #if pyb.millis() - self._t0 >= 1500:
                
                if self._headingCur >= 250:  # no idea if this is correct, this where i stopped
                    # self.sp_left.put(0)
                    # self.sp_right.put(0)
                    self.leftGo.put(0)
                    self.rightGo.put(0)
                    self._sR_start = self.sR_share.get()
                    self._sL_start = self.sL_share.get()
                    self._t0 = pyb.millis()
                    self._state = S7_NEXT
            elif self._state == S7_NEXT:
                self.sp_left.put(0)
                self.sp_right.put(0)
                self.leftGo.put(0)
                self.rightGo.put(0)

            yield self._state