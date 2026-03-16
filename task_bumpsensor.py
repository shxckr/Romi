import micropython, pyb

S0_INIT = micropython.const(0)
S1_WAIT = micropython.const(1)

class task_bumpsensor:
    def __init__(self, left_pins, right_pins,
                 sp_left, sp_right, leftGo, rightGo,
                 collision_mode):
        self.left_pins = left_pins
        self.right_pins = right_pins
        self.collision_mode = collision_mode

        self.sp_left = sp_left
        self.sp_right = sp_right
        self._leftGo = leftGo
        self._rightGo = rightGo

        self._state = S0_INIT

    def _left_hit(self):
        return any(p.value() == 0 for p in self.left_pins)

    def _right_hit(self):
        return any(p.value() == 0 for p in self.right_pins)

    def run(self):
        while True:
            if self._state == S0_INIT:
                self._state = S1_WAIT

            elif self._state == S1_WAIT:
                if not (self._leftGo.get() and self._rightGo.get()):
                    yield 0
                    continue

                if self._left_hit() or self._right_hit():
                    self.collision_mode.put(1)

            yield 0