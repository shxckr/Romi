# ME 405 Term Project
## Project Overview

This project was created by Cal Poly students in ME 405 as their final deliverable for the term project. Students were tasked with programming a Pololu Romi robot to complete a printed track autonomously. The track has five checkpoints that the robot is required to reach, as well as optional bonus challenges. Student groups implemented strategies of their choice to program their robots to reach each checkpoint.

## Robot Overview

The robots in this course use the Romi kit from Pololu and several other Pololu accessories. The microcontroller is an STM32 Nucleo-L476RG board configured to run MicroPython.

## Course Description

The course is mapped by black lines printed on white grid paper. For most of the course, a thick line defines one possible path between checkpoints. Infrared sensors can be used to detect the line, enabling the robot to navigate the course via line following.

The section of the course with no line is referred to as the "parking garage." In this section, the robot must use an alternative means of navigation to maneuver under an aluminum structure. At the end of the parking garage is a wall. The robot must successfully correct its motion after hitting and/or detecting the wall.

To complete the bonus challenges, the robot must move plastic cups into/out of the dashed circles on the course.

## Project Website

The project website is linked [here](file:///C:/Users/saram/OneDrive%20-%20Cal%20Poly/ME%20405/Romi/Romi/docs/build/html/performance.html). Explore the website to learn more about the design of our team's robot (affectionately called BB, or Bumble Bee) and its performance on the course!