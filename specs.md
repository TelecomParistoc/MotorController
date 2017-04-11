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


## "Registers" address and size

The motorboard listens on address 0x12.

Configuration values are placed first, then data and finally targets.
All write-only values are read-as-zero (RAZ).
Writing to a read-only value is implementation-defined, it will defined later.

|Name|Address|Access|Size (in bits)|
|----|-------|------|--------------|
|wheels_gap|0x00|R/W|8|
|ticks_per_cm|0x01|R/W|8|
|angular_trust_threshold|0x02|R/W|16|
|max_linear_acceleration|0x04|R/W|16|
|max_angular_acceleration|0x06|R/W|16|
|cruise_linear_speed|0x08|R/W|16|
|cruise_angular_speed|0x0A|R/W|16|
|current x absolute position|0x10|R|16|
|current y absolute position|0x12|R|16|
|current heading|0x14|R/W|16|
|current right wheel distance|0x16|R|16|
|current left wheel distance|0x18|R|16|
|goal mean distance|0x20|W|16|
|goal heading|0x22|W|16|
|heading distance sync reference|0x24|W|16|
