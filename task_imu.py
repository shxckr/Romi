class task_imu:
    def __init__(self, imu, share_heading, share_yaw_rate):
        self.imu = imu
        self.share_heading = share_heading
        self.share_yaw_rate = share_yaw_rate

    def run(self):
        while True:
            self.share_heading.put(self.imu.heading())
            self.share_yaw_rate.put(self.imu.yaw_rate())
            yield 0
# import pyb

# class task_imu:
#     def __init__(self, imu, share_heading, share_yaw_rate):
#         self.imu = imu
#         self.share_heading = share_heading
#         self.share_yaw_rate = share_yaw_rate
#         self.t_last = pyb.millis()

#     def run(self):
#         while True:
#             self.share_heading.put(self.imu.heading())
#             self.share_yaw_rate.put(self.imu.yaw_rate())

#             if pyb.millis() - self.t_last > 1000:
#                 self.t_last = pyb.millis()
#                 print("cal:", self.imu.get_cal_status())

#             yield