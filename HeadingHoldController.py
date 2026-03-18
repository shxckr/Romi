class HeadingHoldController:
    def __init__(self, heading_share, initHeadSh, target_heading=180.0,
                 base_speed=180.0, kp=2.0, ki=0,min_speed=100.0, max_speed=300.0):
        self.heading_share = heading_share
        self.target_heading = target_heading
        self.base_speed = base_speed
        self.kp = kp
        self.ki = ki
        self.min_speed = min_speed
        self.max_speed = max_speed
        self._initHeadSh = initHeadSh
        self._errSum = 0.0
        self.dt = 0
    def _wrap_angle(self, angle):
        if angle >= 180:
            angle -= 360
        if angle < -180:
            angle += 360
        return angle

    def _clamp(self, value):
        if value > self.max_speed:
            return self.max_speed
        if value < -self.max_speed:
            return -self.max_speed
        return value

    def get_wheel_speeds(self):
        heading = self._wrap_angle(
            self.heading_share.get() - self._initHeadSh.get()
        )

        error = self._wrap_angle(self.target_heading - heading)
        self._errSum += error
        correction = self.kp * error + self.ki * self._errSum
        #if self.base_speed >= 0:
            # forward driving
        left_speed = self.base_speed + correction
        right_speed = self.base_speed - correction
        '''else:
            # reverse driving: flip correction
            left_speed = self.base_speed - correction
            right_speed = self.base_speed + correction'''

        left_speed = self._clamp(left_speed)
        right_speed = self._clamp(right_speed)

        return left_speed, right_speed, error
