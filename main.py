import pyb
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
from task_observer import task_observer
from pyb import Timer, Pin
#import matplotlib.pyplot as plt

# Build all driver objects first
### maybe timer here?
t = Timer(1, freq=10000)   # create shared Timer outside
### Charlie's outline ##########
leftMotor    = motor_driver(t, 1, 'PA8', 'PB4',  'PB10')
rightMotor   = motor_driver(t, 2, 'PA9', 'PB8',  'PB9')
leftEncoder  = encoder(2, 'PA1', 'PA0')
rightEncoder = encoder(3, 'PA6', 'PA7')
sensor_pins = ['PA4', 'PB0', 'PB1', 'PC0', 'PC1', 'PC2', 'PC3']
sensors = LineSensors(*sensor_pins, samples=4)

# --- IMU setup ---
i2c = pyb.I2C(3, pyb.I2C.CONTROLLER, baudrate=400000)   # bus number might be 1 or 2 depending on your wiring
print("I2C scan:", i2c.scan())
# imu = IMU(i2c, addr=0x28)             # 0x28 is common; sometimes it's 0x29
# imu.set_mode(IMU.NDOF)                # fusion mode for heading + gyro feedback
# imu = IMU(i2c, addr=0x28)
# pyb.delay(700)
# imu.set_mode(IMU.NDOF)

imu = IMU(i2c)
pyb.delay(700)

imu.set_mode(IMU.IMUPLUS)   # or NDOF
pyb.delay(100)

CAL = bytes([246, 255, 7, 0, 242, 255, 0, 0, 0, 0, 0, 0,
             255, 255, 254, 255, 1, 0, 232, 3, 0, 0])
imu.set_cal_coeffs(CAL)
print("Loaded saved calibration.")


# -----------------------------------
# Observer matrices (precomputed offline)
# -----------------------------------

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
share_setpoint= Share("f",     name="setpoint")
share_kp.put(0.05)         # default
share_ki.put(0.00)         # default
share_setpoint.put(0.00)   # default
sp_left  = Share("f", name="sp_left")
sp_right = Share("f", name="sp_right")

uL_share = Share("f", name="uL_effort")

uR_share = Share("f", name="uR_effort")
uL_share.put(0.0)
uR_share.put(0.0)

#### new IMU
share_heading  = Share("f", name="heading_deg")
share_yaw_rate = Share("f", name="yaw_rate_dps")
####### Observer class ##################
x_hat_share     = Share("f", name="x_hat")
v_hat_share     = Share("f", name="v_hat")
psi_hat_share   = Share("f", name="psi_hat")
omega_hat_share = Share("f", name="omega_hat")

observer_outputs = {
    'x': x_hat_share,
    'v': v_hat_share,
    'psi': psi_hat_share,
    'omega': omega_hat_share
}

# dataValues    = Queue("f", 30, name="Data Collection Buffer")
# timeValues    = Queue("L", 30, name="Time Buffer")

centroidTime = Queue("L", 30, name="Centroid Time")


leftDataValues   = Queue("f", 30, name="Left Data Buffer")
leftTimeValues   = Queue("L", 30, name="Left Time Buffer")

rightDataValues  = Queue("f", 30, name="Right Data Buffer")
rightTimeValues  = Queue("L", 30, name="Right Time Buffer")

centroidData = Queue("f", 30, name="Centroid")

# Build task class objects (generator functions?)
leftMotorTask  = task_motor(leftMotor, leftEncoder,
                            leftMotorGo, share_kp, share_ki, sp_left,
                            leftDataValues, leftTimeValues,
                            uL_share)

rightMotorTask = task_motor(rightMotor, rightEncoder,
                            rightMotorGo, share_kp, share_ki, sp_right,
                            rightDataValues, rightTimeValues,
                            uR_share)
userTask = task_user(leftMotorGo, rightMotorGo, share_kp, share_ki, share_setpoint,
                     leftDataValues, leftTimeValues,
                     rightDataValues, rightTimeValues,
                     centroidData, centroidTime)
linefollow_task = task_linefollow(sensors,
                                  leftMotorGo, rightMotorGo,
                                  sp_left, sp_right,
                                  centroidData,centroidTime)
observerTask = task_observer(
    leftEncoder, rightEncoder,
    share_heading, share_yaw_rate,
    uL_share, uR_share,      # <-- applied motor effort, not setpoints
    observer_outputs,
    Ts=0.03,
    Ad=A_D,
    Bd=B_D,
    Cd=C_D,
    Dd=D_D
)

### imu
imuTask = task_imu(imu, share_heading, share_yaw_rate)

# Add tasks to task list

### imu

task_list.append(Task(imuTask.run, name="IMU Task",
                      priority=2, period=30, profile=False))

######
task_list.append(Task(leftMotorTask.run, name="Left Mot. Task",  # this is where you call task and set priority/period
                      priority = 1, period = 30, profile=True))  # messing with period
task_list.append(Task(rightMotorTask.run, name="Right Mot. Task",
                      priority = 1, period = 30, profile=True))
task_list.append(Task(userTask.run, name="User Int. Task",
                      priority = 0, period = 0, profile=False))
task_list.append(Task(linefollow_task.run, name="Line Follow Task",
                      priority = 2, period = 30, profile=False))
task_list.append(Task(observerTask.run,
                      name="Observer Task",
                      priority=0,
                      period=30,
                      profile=False))

# Run the garbage collector preemptively
collect()

################ sammie try #################################33
# leftMotor.enable()
# rightMotor.enable()
# # #### run #########
# rightMotor.set_effort(10)  
# leftMotor.set_effort(10) 

# start = pyb.millis()

# while pyb.millis() - start < 1000:
#     rightEncoder.update()
#     leftEncoder.update()

#     print("R pos:", rightEncoder.get_position(),
#           "R vel:", rightEncoder.get_velocity(),
#           " | L pos:", leftEncoder.get_position(),
#           "L vel:", leftEncoder.get_velocity())
    
# # pyb.delay(2000)
# rightMotor.set_effort(0)  
# leftMotor.set_effort(0) 
# pyb.delay(500)

# leftMotor.disable()        # stops and sleeps
# rightMotor.disable() 

# Run the scheduler until the user quits the program with Ctrl-C
while True:
    try:
        task_list.pri_sched()
        # if not centroidData.empty():
        #     print(f"yo homi this is the time{pyb.millis()} and this cen {centroidData.get()}")
            
        
    except KeyboardInterrupt:
        print("Program Terminating")
        leftMotor.disable()
        rightMotor.disable()
        break

#print("\n")
#print(task_list)
#print(show_all())
print("Time,Centroid")

# while not centroidData.empty():
#     t = centroidTime.get()
#     c = centroidData.get()
#     print("{},{}".format(t, c))
########  plotting #########################3


# time_us = []
# velocity = []

# # Paste your serial output into a file called data.txt
# with open("data.txt", "r") as f:
#     for line in f:
#         line = line.strip()
#         if not line:
#             continue
#         if line.startswith("Time"):
#             continue
#         if "," not in line:
#             continue

#         t, v = line.split(",")
#         time_us.append(int(t))
#         velocity.append(float(v))

# # Convert microseconds to seconds
# time_s = [(t - time_us[0]) / 1e6 for t in time_us]

# plt.figure()
# plt.plot(time_s, velocity)
# plt.xlabel("Time (s)")
# plt.ylabel("Velocity (ticks/s)")
# plt.title("Motor Velocity vs Time")
# plt.grid(True)
# plt.show()
