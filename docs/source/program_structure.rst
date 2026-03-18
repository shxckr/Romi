Program Structure
================================

Overview
--------
This project implements a modular control system for the robot using MicroPython on an STM32 Nucleo development board. The microcontroller uses customized firmware files that were provided by Dr. John Ridgely and are available on his ME 405 repository. The program is comprised of several hardware drivers and task files. The drivers are software classes that handle hardware configuruation, allowing for higher-level logic in the tasks. The tasks utilize functions created in the drivers to implement the logic that controls the robot's behavior. All objects are  in a single main file. A scheduler file implements cooperative multitasking, running task objects based on their assigned priority and period. The scheduler driver and task can also be found in the documentation provided by Dr. Ridgely.

Course Strategy
--------------
The program has a specific task designed to navigate the course. This task, "course task," is implemented as a finite state machine. Our team split the course up into several segments, and each segment has its own state in the finite state machine. Each state waits for a specific input to reach a specified condition and trigger the transition to the next state. To start the run, the user hits the blue user button on the Nuelceo. This moves the task from the intialization state to the first run state. The design of the run states and the conditions that trigger transition are detailed in the sections below. Most transition conditions are based on the encoder ticks/ the arc length swept by either the left or right wheel. To avoid error accumulation in the measurement, this value is reset upon transition to the next state.


Diagrams
--------

task diagram

course task STD
maybe other STD

Tasks and Drivers
----------

.. toctree::
   :maxdepth: 1
   :caption: Contents:

   drivers
   tasks