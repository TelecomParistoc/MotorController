#ifndef SETTINGS_H
#define SETTINGS_H

#include "ch.h"

/*
 * Datasheet says that this value should be 900 if angles are measured in radians.
 * But the maximum value remains 5760, whatever the unit selected. And
 * 5760 / 2 * pi = 916.73.
 */
#define ANGLE_MULT 917

/**
 * Distance between the middle of the 2 coding wheels in mm.
 */
extern uint8_t wheels_gap;

/**
 * Number of coding wheel ticks per cm.
 */
extern uint8_t ticks_per_cm;

/**
 * Threshold for angular speed above which we can't trust the IMU anymore.
 * It should be expressed in 1/900 radian.s-1.
 */
extern uint16_t angular_trust_threshold;

/**
 * Maximum linear acceleration authorized, in cm.s-2
 */
extern uint16_t max_linear_acceleration;

/**
 * Maximum angular acceleration authorized, in 1/900 radian.s-2
 */
extern uint16_t max_angular_acceleration;

/**
 * Target cruise linear speed, in cm.s-1
 */
extern uint16_t cruise_linear_speed;

/**
 * Target cruise angular speed, in 1/900 radian.s-1
 */
extern uint16_t cruise_angular_speed;


#endif /* SETTINGS_H */
