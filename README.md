# Overview
This repository contains the code for the MotorController board, developped
by Telecom Robotics.
Below, you'll find the documentation of the components of this project.

# Download
In order to get a working copy of this code, you have to follow some steps. Indeed,
some generic drivers (the IMU driver for now) are included in a submodule.
- clone the repository
- cd in the directory of the repo
- ```$ git submodule init```
- ```$ git submodule update```

At any time when using this repo, if you want to obtain the latest updates of the generic
drivers, you can une ```$ git submodule update --remote```.

# Compilation
The same code is used for both robots. However, as the PID coeffs are different,
two targets are defined:  
For small robot: ```$make small```  
For big robot: ```$make big```

Each of these targets will copy a config_\*\*.h file into *config.h* and then call
the standard ```$make```.
This *config.h* file is used to determine for which robot the code should be
compiled. It basically defines a symbol that is used with #ifdef pre-processor
instructions to select only some parts of the code.
Next times, you can simply run ```$make``` and it will compiles the code for the last
defined target.

# Description
## Communication
This component is in charge of the communication between the central unit of the
robot and the motorboard. It acts as an I2C slave, providing a interface that
looks like memory-mapped registers. For the descriptions of this interface, see
the file [specs.md](./specs.md).

It's composed of 2 files:
   - *i2c_interface.h*
   - *i2c_interface.c*

## Configuration
Some configuration variables are used in order to provide the requested services.
These data are stored in flash (non-volatile memory), so that they can be kept across
reset. When booting up, the firmware loads the values from the flash and initialise
the configuration varaibles with them.  
The I2C interface provides several registers to update these values.
If you want to keep the newly updated values for subsequent execution, it's necessary
to store them in flash. To do so, simply writes any value to the **STORE_DATA_IN_FLASH**
register (see the description of the communication interface to get more info on this
register).

## Coding wheels
This component is responsible of handling the coding wheels inputs. It's basically
an interrupt handler.
It's composed of 2 files:
   - *coding_wheels.h*
   - *coding_wheels.c*

To initialize this driver, call the ```init_coding_wheels()``` function. As
parameter, pass a `coding_wheels_config_t` object. This object must contain the
following information:
   - initial value of each counter (right wheel and left wheel)
   - orientation of each wheel (DIRECT or INDIRECT). This parameter allows the user
     to decide which rotation sense corresponds to a forward movement and thus to
     an increase of the associated counter).

**Note**: It's necessary to enable the ChibiOS EXT driver by setting `HAL_USE_EXT` to
`TRUE` in *halconf.h*

This driver then displays two counters (one for each coding wheel): `right_ticks`
& `left_ticks`.

## Position & orientation
The goal of this component is to provide information regarding the current position
and heading of the robot.

It's composed of several files:
   - *position.h*
   - *position.c*
   - *orientation.h*
   - *orientation.c*
   - *settings.h*
   - *settings.c*

It depends on the following drivers:
   - bno055

## Control
The goal of this component is to manage the low-level part of the robot movements.
It will receive orders from the robot central unit and control the motors to
execute them.

It's composed of 2 files:
   - *control.h*
   - *control.c*

This module defines 2 threads that run in parallel. One is in charge of computing
intermediate orders based on the "high level" orders sent by the master and on the
ettings defined (max acceleration, cruise speed...). The other is responsible of
applying a PID on the motors command based on the current order and on data read
from sensors (coding wheels and IMU for the moment).

# TODO
  - improve position computation (in position.c:update_position)
  - find a way to give meaningful value (unit) to speed and acceleration (in control.c)
  - set proper PID coeffs
