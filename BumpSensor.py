'''
bump_driver.py

Romi Bumper Switch Kit driver (Pololu #3674) with ISR callbacks + software debounce.

Debounce strategy (per ME405 Week 10 PDF):
- Each falling edge interrupt (active-low press) triggers an ISR.
- ISR disables that interrupt line and records it in a debounce mask.
- A periodic task (update()) runs every debounce interval (~10–20ms),
  re-enables any lines that were disabled in the previous interval.

Assumptions:
- MicroPython on STM32 with `pyb` module available.
- Switches are wired active-low with pull-ups enabled.
'''

from pyb import Pin, ExtInt, disable_irq, enable_irq
from array import array


class BumpDriver:
    """
    Driver for one or more bumper switches using external interrupts + debounce.

    Parameters
    ----------
    pins : list[pyb.Pin]
        List of Pin objects for the bumper switches.
        Example: [Pin('PC0', Pin.IN), Pin('PC1', Pin.IN), ...]
    callback : function
        User function called on a press event: callback(isr_line, pin_obj).
        IMPORTANT: Keep it short/fast (it runs in interrupt context).
    debounce_ms : int
        Debounce window. Call update() at about this interval.
    pull : int
        Pin pull configuration (defaults to Pin.PULL_UP since bumpers are active-low).
    """

    def __init__(self, pins, callback, debounce_ms=20):
        
        self._pins = pins
        self._user_cb = callback
        self.debounce_ms = int(debounce_ms)

        # Map ISR line number (0-15) -> (ExtInt object, Pin object)
        self._lines = {}

        # Debounce masks: [current_window_mask, previous_window_mask]
        self._db_mask = array("H", [0x0000, 0x0000])
        for p in self._pins:   
            ext = ExtInt(p, ExtInt.IRQ_FALLING, Pin.PULL_UP, self._isr)
            self._lines[p.pin()] = (ext, p)

    def _isr(self, isr_line):
        """
        ISR called by ExtInt. `isr_line` is the interrupt line number (0..15).
        """
        # Mark this line as having triggered in the current debounce window
        self._db_mask[0] |= (1 << isr_line)

        # Disable further interrupts on this line until update() re-enables it
        ext, pin_obj = self._lines[isr_line]
        ext.disable()

        # Call user callback (keep it short!)
        if self._user_cb:
            try:
                self._user_cb(isr_line, pin_obj)
            except Exception:
                # Avoid crashing hard in interrupt context
                pass

    def update(self):
        """
        Call this periodically (e.g., every 10–20ms) to re-enable lines that were
        disabled during the previous debounce window.

        This matches the PDF pattern:
        - Re-enable any channels whose bit was set in _db_mask[1]
        - Atomically shift current->previous and clear current
        """
        # Re-enable any lines that were pending from previous window
        prev = self._db_mask[1]
        if prev:
            for isr_line in range(16):
                if prev & (1 << isr_line):
                    ext, _ = self._lines.get(isr_line, (None, None))
                    if ext:
                        ext.enable()

        # Critical section: shift masks
        irq_state = disable_irq()
        self._db_mask[1], self._db_mask[0] = self._db_mask[0], 0x0000
        enable_irq(irq_state)

    def enable_all(self):
        """Enable all configured interrupt lines."""
        for isr_line, (ext, _) in self._lines.items():
            ext.enable()

        irq_state = disable_irq()
        self._db_mask[0] = 0x0000
        self._db_mask[1] = 0x0000
        enable_irq(irq_state)

    def disable_all(self):
        """Disable all configured interrupt lines."""
        for isr_line, (ext, _) in self._lines.items():
            ext.disable()

    def read(self):
        """
        Read current raw pin states.

        Returns dict: {isr_line: 0/1} where 0 means pressed (active-low).
        """
        return {line: pin.value() for line, (_, pin) in self._lines.items()}