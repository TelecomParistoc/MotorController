# Overview
This repository contains the code for the MotorController board, developped
by Telecom Robotics.
Below, you'll find the documentation of the components of this project.

# Description
## Communication
This component is in charge of the communication between the central unit of the
robot and the motorboard. It acts as an I2C slave, providing a interface that
looks like memory-mapped registers. For the descriptions of this interface, see
the file specs.md.

It's composed of 2 files:
   - i2c_interface.h
   - i2c_interface.c

In order to include the I2C slave port of the ChibiOS I2C driver, this driver has
been replaced by the extended one. This replacement should be done automatically
when you clone the repository and the sub repositories. In case it doesn't work,
download the extended I2C driver [here](http://www.chibios.com/forum/download/file.php?id=1131&sid=bc734dbc0c5a781fb2b4d3acb146bdec).
Then, place all the files of this driver in the ChibiOS directory. Some of them
replace existing files and some are new.

## Position & orientation
The goal of this component is to provide information regarding the current position
and heading of the robot.

It's composed of several files:
   - position.h
   - position.c
   - orientation.h
   - orientation.c
   - settings.h
   - settings.c

It depends on the following drivers:
   - bno055

## Control
The goal of this component is to manage the low-level part of the robot movements.
It will receive orders from the robot central unit and control the motors to
execute them.
This component is not developed for the moment.
