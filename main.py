from motor_driver import motor_driver
from encoder      import encoder
from task_motor   import task_motor
from task_user    import task_user
from linesensors import LineSensors
from task_linefollow import task_linefollow
from task_share   import Share, Queue, show_all
from cotask       import Task, task_list
from gc           import collect
from IMU          import IMU
from task_imu     import task_imu
from task_bumpsensor    import task_bumpsensor
from BumpSensor    import BumpDriver
from task_observer import task_observer
from pyb import Timer, Pin, delay, USB_VCP, I2C, millis
from task_courseTest import task_course
from task_UserButton import task_UButton

collect()
# Build all driver objects first
t = Timer(1, freq=10000)   # create shared Timer outside
leftMotor    = motor_driver(t, 1, 'PA8', 'PB4',  'PB10')
rightMotor   = motor_driver(t, 2, 'PA9', 'PB8',  'PB9')
leftEncoder  = encoder(2, 'PA1', 'PA0')
rightEncoder = encoder(3, 'PA6', 'PA7')
sensor_pins = ['PA4', 'PB0', 'PB1', 'PC4', 'PC5', 'PC2', 'PC3']
sensors = LineSensors(sensor_pins, samples=4)
L0 = Pin('PB12', Pin.IN, pull=Pin.PULL_UP)  # bumper 3
L1 = Pin('PC6', Pin.IN, pull=Pin.PULL_UP) # bumper 4
L2 = Pin('PC8', Pin.IN, pull=Pin.PULL_UP) # bumper 5

# RIGHT bump switches (3)
R0 = Pin('PC11', Pin.IN, pull=Pin.PULL_UP) # bump 0
R1 = Pin('PC12', Pin.IN, pull=Pin.PULL_UP) #bump 1
R2 = Pin('PC10', Pin.IN, pull=Pin.PULL_UP) #bum[ 2]
bump_left_pins  = [L0, L1, L2]
bump_right_pins = [R0, R1, R2]
# User button pin
USER_BUTTON = Pin('PC13', Pin.IN, pull=Pin.PULL_UP)

# --- IMU setup ---
i2c = I2C(3, I2C.CONTROLLER, baudrate=400000)   
print("I2C scan:", i2c.scan())

imu = IMU(i2c)
delay(700)

imu.set_mode(IMU.IMUPLUS)   # or NDOF
delay(100)

CAL = bytes([246, 255, 7, 0, 242, 255, 0, 0, 0, 0, 0, 0,
             255, 255, 254, 255, 1, 0, 232, 3, 0, 0])
imu.set_cal_coeffs(CAL)
print("Loaded saved calibration.")
# Observer matrices (precomputed offline)
A_D = ([
[0.6132, 0.0000, 0.3113, 0.3113],
[0.0000, 0.0005, 0.0000, 0.0000],
[-0.1512, 0.0000, 0.2359, 0.2350],
[-0.1512, 0.0000, 0.2350, 0.2359]
])

B_D = ([
[0.3939, 0.3939, 0.1934, 0.1934, -0.0000, 0.0000],
[0.0000, 0.0000, -0.0071, 0.0071, 0.0001, 0.0039],
[0.9063, 0.6055, 0.0756, 0.0756, -0.0000, -1.8257],
[0.6055, 0.9063, 0.0756, 0.0756, -0.0000, 1.8257]
])

C_D = ([
   [1.0000, -70.0000, 0.0000, 0.0000],
   [1.0000,  70.0000, 0.0000, 0.0000],
   [0.0000,   1.0000, 0.0000, 0.0000],
   [0.0000,   0.0000, -0.2500, 0.2500]
])


D_D =([
[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
])

# Build shares and queues
leftMotorGo   = Share("B",     name="Left Mot. Go Flag")
rightMotorGo  = Share("B",     name="Right Mot. Go Flag")
share_kp      = Share("f",     name="kp value")
share_ki      = Share("f",     name="ki value")
share_kp.put(0.04)         # default
share_ki.put(0.01)         # default
share_calD = Share("B", name="calibrate dark flag")
share_calL = Share("B", name="calibrate light flag")
share_lineKp = Share("f", name="line following Kp")
share_lineKp.put(375)
share_lineKi = Share("f", name="line following Ki")
share_lineKi.put(0.1)
sp_left  = Share("f", name="sp_left")
sp_right = Share("f", name="sp_right")
uL_share = Share("f", name="uL_effort")
uR_share = Share("f", name="uR_effort")
DoneCalSh = Share("B", name="Calibration done flag")
uL_share.put(0.0)
uR_share.put(0.0)
UB_share = Share("B", name="User Button Flag")
#### new IMU
share_heading  = Share("f", name="heading_deg")
share_yaw_rate = Share("f", name="yaw_rate_dps")
####### Observer class ##################
s_hat_share     = Share("f", name="s_hat")
psi_hat_share     = Share("f", name="psi_hat")
omega_L_share   = Share("f", name="omega_L_hat")
omega_R_share = Share("f", name="omega_R_hat")
X_share         = Share("f", name="X")
Y_share         = Share("f", name="Y")
collision_mode = Share("B", name="Collision Mode")  # 0=normal, 1=bumper override

yolo_mode = Share("B", name="YOLO Mode")
yolo_mode.put(0)
ser = USB_VCP()
sL_share = Share("f", name="sL")
sL_share.put(0.0)
sR_share = Share("f", name="sR")
sR_share.put(0.0)
observer_outputs = {
    's': s_hat_share,
    'psi': psi_hat_share,
    'omega_L': omega_L_share,
    'omega_R': omega_R_share,
    'X': X_share,
    'Y': Y_share
}

centroidTime = Queue("L", 30, name="Centroid Time")


leftDataValues   = Queue("f", 60, name="Left Data Buffer")
leftTimeValues   = Queue("L", 60, name="Left Time Buffer")

rightDataValues  = Queue("f", 60, name="Right Data Buffer")
rightTimeValues  = Queue("L", 60, name="Right Time Buffer")

centroidData = Queue("f", 30, overwrite=True, name="Centroid")
statePredTime = Queue("L", 30, overwrite=True, name="Prediction Time")
statePredX = Queue("f", 100, overwrite=True, name="Prediction Global X")
statePredY = Queue("f", 100, overwrite=True, name="Prediction Global Y")
statePredSL = Queue("f", 30, overwrite=True, name="Prediction Arc Length Left")
stateMeasXL = Queue("f", 30, overwrite=True, name="Measured Arc Length Left")
statetime = Queue("f", 100, overwrite=True, name="State Time")
sL_yhat = Queue("f", 100, overwrite=True, name="Prediction sL")
sL_meas = Queue("f", 100, overwrite=True, name="Measured sL")
initHeadSh = Share("f", name="Initial heading")

# Build task class objects (generator functions?)
leftMotorTask  = task_motor(leftMotor, leftEncoder,
                            leftMotorGo, share_kp, share_ki, sp_left,
                            leftDataValues, leftTimeValues,
                            uL_share)

rightMotorTask = task_motor(rightMotor, rightEncoder,
                            rightMotorGo, share_kp, share_ki, sp_right,
                            rightDataValues, rightTimeValues,
                            uR_share)
userTask = task_user(leftMotorGo, rightMotorGo, share_kp, share_ki,
                     leftDataValues, leftTimeValues,
                     rightDataValues, rightTimeValues, centroidData, centroidTime,
                     statePredX, statePredY, sL_yhat, sL_meas, statetime, share_calL, share_calD, 
                     ser,share_lineKp, share_lineKi, sensors, DoneCalSh, share_heading)
linefollow_task = task_linefollow(
    sensors,
    leftMotorGo, rightMotorGo,
    sp_left, sp_right,
    centroidData, centroidTime, share_calL, share_calD,
    collision_mode, yolo_mode, share_lineKp, share_lineKi, DoneCalSh       # <-- add
)

observerTask = task_observer(
    leftEncoder, rightEncoder,
    share_heading, share_yaw_rate,
    uL_share, uR_share,
    observer_outputs, statePredX, statePredY,
    leftMotorGo, rightMotorGo,
    sL_yhat, sL_meas, statetime,
    sL_share, sR_share,
     Ts=0.03, # Ts
    Ad=A_D,
    Bd=B_D,
    Cd=C_D,
    Dd=D_D, )

### imu
imuTask = task_imu(imu, share_heading, share_yaw_rate)

# Add tasks to task list
bumpTask = task_bumpsensor(
    bump_left_pins, bump_right_pins,
    sp_left, sp_right, leftMotorGo, rightMotorGo,
    collision_mode      # <-- add
)
courseTask = task_course(
    sL_share,
    sR_share,
    collision_mode,
    yolo_mode,
    sp_left,
    sp_right,
    leftMotorGo,
    rightMotorGo,
    sensors,
    share_heading,
    share_yaw_rate,
    ser,
    initHeadSh,
    UB_share
)
userButtonTask = task_UButton(USER_BUTTON, UB_share)
### imu
task_list.append(Task(imuTask.run, name="IMU Task",
                      priority=1, period=30, profile=False))
task_list.append(Task(leftMotorTask.run, name="Left Mot. Task",  # this is where you call task and set priority/period
                      priority = 2, period = 30, profile=True))  # messing with period
task_list.append(Task(rightMotorTask.run, name="Right Mot. Task",
                      priority = 2, period = 30, profile=True))
task_list.append(Task(userTask.run, name="User Int. Task",
                      priority = 0, period = 100, profile=False))
task_list.append(Task(linefollow_task.run, name="Line Follow Task",
                      priority = 2, period = 30, profile=False))
task_list.append(Task(observerTask.run,
                      name="Observer Task",
                      priority=1,
                      period=20,
                      profile=False))
task_list.append(Task(bumpTask.run, name="Bump Task", priority=3, period=20))
task_list.append(Task(courseTask.run,
                      name="Course Task",
                      priority=3,
                      period=30))
task_list.append(Task(userButtonTask.run, name="User Button Task", priority=2, period=100))

# Run the garbage collector preemptively
collect()

# Run the scheduler until the user quits the program with Ctrl-C
last = millis()
while True:
    try:
        task_list.pri_sched()
    except KeyboardInterrupt:
        print("Program Terminating")
        leftMotor.disable()
        rightMotor.disable()
        break
