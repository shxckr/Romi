class task_imu:
    '''
    This task reads the from the IMU and updates the heading and yaw rate shares with the readings
    '''
    def __init__(self, imu, share_heading, share_yaw_rate):
        '''
        Initializes task_imu object

        Arguements:
        imu: a pre-initialized IMU object that communicates with the STM32L476RG through I2C
        share_heading: a share that contains the most recent reading of the IMU heading
        share_yaw_rate: a share that contains the most recent reading of the IMU yaw rate
        '''
        self.imu = imu
        self.share_heading = share_heading
        self.share_yaw_rate = share_yaw_rate

    def run(self):
        '''
        Runs one iteration of the task_imu and updates the heading and yaw rate shares
        '''
        while True:
            self.share_heading.put(self.imu.heading())
            self.share_yaw_rate.put(self.imu.yaw_rate())
            yield
