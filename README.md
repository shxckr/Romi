# Romi
This repository holds the code created by Cal Poly students in ME405, Winter 2026 (group mecha32).
The documentation pages are at ... (Web link)

Scheduler Files, Firmware, and Other Support Files (modified from Dr. Ridgely's ME405 Repository)

    The scheduler files are used to implement cooperative multitasking in MicroPython. The cotask.py and task_share.py modules, provided by the instructor, create this structure.

    The file firmware.bin contains a custom version of MicroPython for use only on an STM32L476RG Nucleo. It supports extra UARTs, DAC, and the use of the USB-OTG connector on the Shoe of Brian (see the github-pages documents) to connect the /flash directory as a USB drive. CAN is not supported, as the pins needed are used by the USB-OTG connector.

    This firmware file also contains MicroPython-ulab, the NumPy/SciPy partial workalike library found at https://github.com/v923z/micropython-ulab, and cqueue, a module hosted in this repository which contains fast and efficient queues to transfer data between tasks. 

    src/nb_input.py contains a class which implements non-blocking input from a serial port on a microcontroller running MicroPython.


Drivers
    The 