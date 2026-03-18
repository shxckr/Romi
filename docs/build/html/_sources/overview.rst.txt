Romi mecha32 Robot Documentation
================================

Intro
--------

This project implements a modular control system for the Pololu
Romi robot using MicroPython.

Subsystems
----------

The robot software is divided into several subsystems:

- Motor driver
- Encoder interface
- Line sensors
- IMU driver
- Task scheduling

Repository Structure
--------------------

.. code-block:: text

   Romi/
   ├── motor_driver.py
   ├── encoder.py
   ├── linesensors.py
   ├── IMU.py
   └── tasks/

Module Reference
----------------

.. toctree::
   :maxdepth: 3
   :caption: Contents:

   overview
   drivers