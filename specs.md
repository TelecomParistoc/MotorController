# MotorController v2
firware of the dedidated motion processor.

## Goals

The motion processor is required to :
* keep track of the current absolute position and heading of the robot
* provide an high level interface to accurately control the motion of the robot

The robot should be able to go to any goal postion and heading using a trajectory composed of two arcs and a straight line.
Here is an example :

![trajectory](specs/trajectory.jpeg)

Because to the robot is not capable of infinite linear nor angular acceleration, maximum accelerations should be defined and
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
holding its current poistion.

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
* heading distance sync reference (read/write)
<br>
* linear PID coefficients (read/write, flash stored)
* angular PID coefficients (read/write, flash stored)
