Program Structure
================================

Overview
--------
This project implements a modular control system for the robot using MicroPython on an STM32 Nucleo development board. The microcontroller uses customized firmware files that were provided by Dr. John Ridgely and are available on his ME 405 repository. The program is comprised of several hardware drivers and task files. The drivers are software classes that handle hardware configuruation, allowing for higher-level logic in the tasks. The tasks utilize functions created in the drivers to implement the logic that controls the robot's behavior. All objects are  in a single main file. A scheduler file implements cooperative multitasking, running task objects based on their assigned priority and period. The scheduler driver and task can also be found in the documentation provided by Dr. Ridgely.

Course Strategy
--------------
BB uses a task to navigate the course. This task, titled "course task" is implemented as a finite state machine with separate states for various segments of the course.


Diagrams
--------

task diagram

course task STM
maybe other STM

Tasks and Drivers
----------

.. toctree::
   :maxdepth: 1
   :caption: Contents:

   drivers
   tasks