class HeadingHoldController:
    """PI controller for maintaining a desired robot heading.

    This controller computes left and right wheel speeds to keep the robot
    aligned with a target heading using proportional-integral control.
    Heading error is wrapped to the range [-180, 180] degrees.
    """
    def __init__(self, heading_share, initHeadSh, target_heading=180.0,
                 base_speed=180.0, kp=2.0, ki=0,min_speed=100.0, max_speed=300.0):
        """Initialize the heading hold controller.

        Args:
            heading_share: Shared variable containing the current heading (degrees).
            initHeadSh: Shared variable storing the initial heading offset (degrees).
            target_heading: Desired heading relative to the initial heading (degrees).
            base_speed: Nominal forward speed for both wheels.
            kp: Proportional gain.
            ki: Integral gain.
            min_speed: Minimum allowable wheel speed (currently unused).
            max_speed: Maximum magnitude of wheel speed.
        """
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
        """Wrap an angle to the range [-180, 180] degrees.

        Args:
            angle: Input angle in degrees.

        Returns:
            Wrapped angle in the range [-180, 180].
        """
        if angle >= 180:
            angle -= 360
        if angle < -180:
            angle += 360
        return angle

    def _clamp(self, value):
        """Clamp a value to the allowed motor speed range.

        Args:
            value: Input speed command.

        Returns:
            Clamped speed within [-max_speed, max_speed].
        """
        if value > self.max_speed:
            return self.max_speed
        if value < -self.max_speed:
            return -self.max_speed
        return value

    def get_wheel_speeds(self):
        """Compute wheel speeds to correct heading error.

        The controller calculates heading error relative to the initial
        heading, applies PI control, and adjusts left/right wheel speeds
        accordingly.

        Returns:
            tuple:
                left_speed (float): Commanded speed for left wheel.
                right_speed (float): Commanded speed for right wheel.
                error (float): Heading error in degrees.
        """
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
