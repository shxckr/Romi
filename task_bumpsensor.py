import micropython, pyb

S0_INIT = micropython.const(0)
S1_WAIT = micropython.const(1)
S2_LEFT = micropython.const(2)
S3_RIGHT = micropython.const(3)
S4_BOTH = micropython.const(4)
S5_COOLDOWN = micropython.const(5)

class task_bumpsensor:
    def __init__(self, left_pins, right_pins,
                 sp_left, sp_right, leftGo, rightGo,
                 collision_mode,
                 backup_ms=1000, turn_ms=1500, cooldown_ms=600):
        self.left_pins = left_pins      # list of 3 Pin objects
        self.right_pins = right_pins    # list of 3 Pin objects
        self.collision_mode = collision_mode

        self.sp_left = sp_left
        self.sp_right = sp_right
        self._leftGo = leftGo
        self._rightGo = rightGo

        self.backup_ms = backup_ms
        self.turn_ms = turn_ms
        self.cooldown_ms = cooldown_ms

        self._state = S0_INIT
        self._t0 = 0

    def _left_hit(self):
        # active-low: pressed == 0
        return any(p.value() == 0 for p in self.left_pins)

    def _right_hit(self):
        return any(p.value() == 0 for p in self.right_pins)

    def run(self):
        while True:
            if self._state == S0_INIT:
               # self.sp_left.put(0)
                #self.sp_right.put(0)
                self.collision_mode.put(0)
                self._state = S1_WAIT

            #elif self._state == S1_WAIT:
            elif self._state == S1_WAIT:
                if not (self._leftGo.get() and self._rightGo.get()):
        # robot not armed/running yet
                    self.collision_mode.put(0)
                    yield 0
                    continue
                L = self._left_hit()
                R = self._right_hit()
                print("Left pins:", [p.value() for p in self.left_pins])
                print("Right pins:", [p.value() for p in self.right_pins])
                print("L:", L, "R:", R)

                if L and R:
                    self.collision_mode.put(1)
                    self._state = S4_BOTH
                elif L:
                    self.collision_mode.put(1)
                    self._state = S2_LEFT
                elif R:
                    self.collision_mode.put(1)
                    self._state = S3_RIGHT

            elif self._state == S2_LEFT:
                # backup
                self.sp_left.put(-180); self.sp_right.put(-180)
                self._t0 = pyb.millis()
                self._state = S2_LEFT + 10

            elif self._state == S2_LEFT + 10:
                if pyb.millis() - self._t0 >= self.backup_ms:
                    # turn right
                    self.sp_left.put(0); self.sp_right.put(-180)
                    self._t0 = pyb.millis()
                    self._state = S2_LEFT + 20

            elif self._state == S2_LEFT + 20:
                if pyb.millis() - self._t0 >= self.turn_ms:
                    self.sp_left.put(0); self.sp_right.put(0)
                    self._t0 = pyb.millis()
                    self._state = S5_COOLDOWN

            elif self._state == S3_RIGHT:
                # backup
                self.sp_left.put(-180); self.sp_right.put(-180)
                self._t0 = pyb.millis()
                self._state = S3_RIGHT + 10

            elif self._state == S3_RIGHT + 10:
                if pyb.millis() - self._t0 >= self.backup_ms:
                    # turn left
                    self.sp_left.put(-180); self.sp_right.put(0)
                    self._t0 = pyb.millis()
                    self._state = S3_RIGHT + 20

            elif self._state == S3_RIGHT + 20:
                if pyb.millis() - self._t0 >= self.turn_ms:
                    self.sp_left.put(0); self.sp_right.put(0)
                    self._t0 = pyb.millis()
                    self._state = S5_COOLDOWN

            elif self._state == S4_BOTH:
                # backup longer
                self.sp_left.put(-180); self.sp_right.put(-180)
                self._t0 = pyb.millis()
                self._state = S4_BOTH + 10

            elif self._state == S4_BOTH + 10:
                if pyb.millis() - self._t0 >= self.backup_ms + 100:
                    # pick a direction (right turn here)
                    self.sp_left.put(-180); self.sp_right.put(0)
                    self._t0 = pyb.millis()
                    self._state = S4_BOTH + 20

            elif self._state == S4_BOTH + 20:
                if pyb.millis() - self._t0 >= self.turn_ms + 50:
                    self.sp_left.put(0); self.sp_right.put(0)
                    self._t0 = pyb.millis()
                    self._state = S5_COOLDOWN

            elif self._state == S5_COOLDOWN:
                if pyb.millis() - self._t0 >= self.cooldown_ms:
                    self.collision_mode.put(0)
                    self._state = S1_WAIT

            yield 0