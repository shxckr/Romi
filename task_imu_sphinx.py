class task_imu:
    def __init__(self, imu, share_heading, share_yaw_rate):
        self.imu = imu
        self.share_heading = share_heading
        self.share_yaw_rate = share_yaw_rate

    def run(self):
        while True:
            self.share_heading.put(self.imu.heading())
            self.share_yaw_rate.put(self.imu.yaw_rate())
            yield