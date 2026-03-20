class task_imu:
    '''file is used to interface between the imu driver and other tasks by 
    initializing imu and putting its readings into 
    '''
    def __init__(self, imu, share_heading, share_yaw_rate):
        '''initializes the imu task'''
        self.imu = imu
        self.share_heading = share_heading
        self.share_yaw_rate = share_yaw_rate

    def run(self):
        '''runs on instance of the imu task'''
        while True:
            self.share_heading.put(self.imu.heading())
            self.share_yaw_rate.put(self.imu.yaw_rate())
            yield