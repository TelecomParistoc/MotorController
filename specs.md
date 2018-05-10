# MotorController v2
firmware of the dedicated motion processor.

## Goals

The motion processor is required to :
* keep track of the current absolute position and heading of the robot
* provide an high level interface to accurately control the motion of the robot

The robot should be able to go to any goal position and heading using a trajectory composed of two arcs and a straight line.
Here is an example :

![trajectory](specs/trajectory.jpeg)

Because the robot is not capable of infinite linear nor angular acceleration, maximum accelerations should be defined and
the speed profiles should look like (for example) :

![speeds](specs/speeds.png)

Given the intricate relation between motion and high level considerations, the computation of the trajectory won't be performed by the
motion coprocessor. It will receive position control commands :
* maximum linear and angular accelerations
* linear and angular cruise speeds
* goal mean distance (mean distance being the mean between the distance of the two wheels)
* goal heading, and eventually a reference mean distance to start the rotation. This should allow for a precise synchronization between
translation and rotation.

This way of controlling motion provide a flexible interface without overloading the I2C bus through which the commands are transmitted,
nor requiring any real time capabilities from the master.

The MotionController will provide a **position control**, so that the robot is either moving toward its goal or
holding its current position.

The real position should also be tracked, using a fusion of the data from the encoder wheels, the IMU and radio measurements.
A special attention should be given to the IMU response time. Indeed when rotating, it is suspected that the heading measurement lags.

## Interface

The MotionController should provide read/write data to the I2C master mapped as 8 or 16 bits registers :

* current x absolute position (read)
* current y absolute position (read)
* current heading (read/write)

<br>

* current right wheel distance (read/write)
* current left wheel distance (read/write)

<br>

* maximum accelaration (read/write)
* maximum angular accelaration (read/write)
* cruise speed (read/write)
* cruise angular speed (read/write)
* goal mean distance (read/write)
* goal heading (read/write)
* heading distance sync reference (read/write)<br>

<br>

* linear PID coefficients (read/write, flash stored)
* angular PID coefficients (read/write, flash stored)

Any distance is in mm.
Angle range is [0, 360].

## "Registers" address and size

The motorboard listens on address 0x12 (device address).

Configuration values are placed first, then data and finally targets.
All write-only values are read-as-zero (RAZ).
Writing to a read-only value is implementation-defined, it will defined later.
For 32 bits value, split into 2 16-bit register, the LOW register must always be
read/written first. Failing to follow this rule will lead to invalid data.


|Name|Address|Access|Size (in bits)|
|----|-------|------|--------------|
|wheels_gap (in mm)|0x00|R/W|uint16|
|ticks_per_m|0x02|R/W|uint16|
|angular_trust_threshold [absolute value] (in °.s-1)|0x04|R/W|uint16|
|max_linear_acceleration (in mm.s-2)|0x06|R/W|uint16|
|max_angular_acceleration (in °.s-2)|0x08|R/W|uint16|
|cruise_linear_speed (in mm.s-1)|0x0a|R/W|uint16|
|cruise_angular_speed (in °.s-1)|0x0c|R/W|uint16|
|linear p coefficient|0x0e|R/W|uint16|
|linear i coefficient|0x10|R/W|uint16|
|linear d coefficient|0x12|R/W|uint16|
|angular p coefficient|0x14|R/W|uint16|
|angular i coefficient|0x16|R/W|uint16|
|angular d coefficient|0x18|R/W|uint16|
|motor left forward sense|0x1a|R/W|uint8|
|motor right forward sense|0x1b|R/W|uint8|
|coding wheel left initial ticks|0x1c|R/W|uint32|
|coding wheel right initial ticks|0x20|R/W|uint32|
|coding wheel left orientation|0x24|R/W|uint8|
|coding wheel right orientation|0x25|R/W|uint8|
|linear allowance (in mm)|0x26|R/W|uint16|
|angular allowance (in internal unit)|0x28|R/W|uint16|
|left motor coefficient|0x30|R/W|uint16|
|right motor coefficient|0x32|R/W|uint16|
|store config data in flash|0x30|W|uint8|
|Use wall or obstacle to reset orientation|0x31|W|uint8|
|master stop for motors|0xa8|R/W|uint8|
|current x absolute position|0x80|R/W|uint16|
|current y absolute position|0x84|R/W|uint16|
|current right wheel distance|0x88|R/W|uint32|
|current left wheel distance|0x8c|R/W|uint32|
|current heading|0x90|R/W|uint16|
|current mean distance (in mm)|0x92|R|uint32|
|translation ended|0x96|R|uint8|
|rotation ended|0x97|R|uint8|
|goal mean distance (in mm)|0xa0|W|uint32|
|goal heading (in °)|0xa4|W|uint16|
|heading distance sync reference (in mm)|0xa6|R/W|uint16|
|Direction for reset orientation|0xa9|R/W|uint8|
|New orientation for reset orientation|0xaa|R/W|uint16|

Device address: 0x12
